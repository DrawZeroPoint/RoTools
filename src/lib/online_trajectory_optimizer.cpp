//
// Created by smart on 2021/9/6.
//

#include "roport/online_trajectory_optimizer.h"
#include "roport/common.h"

namespace rotools {

RuckigOptimizer::RuckigOptimizer(int dof,
                                 const std::vector<double>& max_vel,
                                 const std::vector<double>& max_acc,
                                 const std::vector<double>& max_jerk,
                                 double frequency,
                                 double reduce_ratio) {
  if (dof > capacity_) {
    // The capacity can be raised if needed
    ROS_ERROR("RoTools: DOF %d exceed the capacity (%d) of the online trajectory optimizer", dof, capacity_);
    return;
  }
  assert(reduce_ratio > 0 && frequency > 0);
  assert(!max_vel.empty() && max_vel.size() == max_acc.size() && max_vel.size() == max_jerk.size());

  dof_ = new int(dof);
  frequency_ = new double(frequency);

  is_initial_state_set_ = new bool(false);
  is_target_state_set_ = new bool(false);
  start_ = new std::chrono::steady_clock::time_point;

  trajectory_generator_ = new ruckig::Ruckig<capacity_>(1. / *frequency_);
  input_param_ = new ruckig::InputParameter<capacity_>();
  output_param_ = new ruckig::OutputParameter<capacity_>();

  for (size_t i = 0; i < capacity_; i += 1) {
    if (i < *dof_) {
      input_param_->max_velocity[i] = reduce_ratio * max_vel[i];
      input_param_->max_acceleration[i] = reduce_ratio * max_acc[i];
      input_param_->max_jerk[i] = reduce_ratio * max_jerk[i];
    } else {
      // Setting non-zero limits make sure Ruckig receive valid limits and not producing zeros for all cmd.
      input_param_->max_velocity[i] = reduce_ratio;
      input_param_->max_acceleration[i] = reduce_ratio;
      input_param_->max_jerk[i] = reduce_ratio;
    }
  }
}

RuckigOptimizer::~RuckigOptimizer() {
  delete trajectory_generator_;
  delete input_param_;
  delete output_param_;
  delete start_;
}

void RuckigOptimizer::setInitialState(const sensor_msgs::JointState& msg) {
  assert(*is_target_state_set_);
  std::array<double, capacity_> position{};
  std::array<double, capacity_> velocity{};
  std::array<double, capacity_> acceleration{};
  std::copy(msg.position.begin(), msg.position.end(), position.begin());
  std::copy(msg.velocity.begin(), msg.velocity.end(), velocity.begin());
  input_param_->current_position = position;
  input_param_->current_velocity = velocity;
  input_param_->current_acceleration = acceleration;
  *is_initial_state_set_ = true;
  *start_ = std::chrono::steady_clock::now();
}

void RuckigOptimizer::setTargetState(const sensor_msgs::JointState& msg) {
  std::array<double, capacity_> position{};
  std::array<double, capacity_> velocity{};
  std::copy(msg.position.begin(), msg.position.end(), position.begin());
  std::copy(msg.velocity.begin(), msg.velocity.end(), velocity.begin());
  input_param_->target_position = position;
  input_param_->target_velocity = velocity;
  *is_target_state_set_ = true;
}

void RuckigOptimizer::getTargetPosition(std::vector<double>& q_d) {
  assert(*is_target_state_set_);
  q_d.resize(*dof_);
  std::copy(input_param_->target_position.begin(), input_param_->target_position.begin() + *dof_, q_d.begin());
}

bool RuckigOptimizer::update(std::vector<double>& q_cmd, std::vector<double>& dq_cmd) {
  assert(*is_initial_state_set_ && *is_target_state_set_);

  auto interval = std::chrono::steady_clock::now() - *start_;
  auto i_ms = std::chrono::duration_cast<std::chrono::milliseconds>(interval);
  const unsigned long kSteps = std::max<unsigned long>(i_ms.count(), 1);
  for (unsigned long i = 0; i < kSteps; i++) {
    auto result = trajectory_generator_->update(*input_param_, *output_param_);
    if (result != ruckig::Working && result != ruckig::Finished) {
      ROS_ERROR_STREAM("Ruckig is on error " << result << ", make sure the max limits are not close to 0");
      roport::logWarningList<std::array<double, capacity_>>(output_param_->new_position, "Output new position");
      roport::logWarningList<std::array<double, capacity_>>(output_param_->new_velocity, "Output new velocity");
      throw std::runtime_error("Ruckig failed");
    }
    output_param_->pass_to_input(*input_param_);
  }
  std::array<double, capacity_> new_position = output_param_->new_position;
  std::array<double, capacity_> new_velocity = output_param_->new_velocity;

  q_cmd.resize(*dof_);
  dq_cmd.resize(*dof_);
  std::copy(new_position.begin(), new_position.begin() + *dof_, q_cmd.begin());
  std::copy(new_velocity.begin(), new_velocity.begin() + *dof_, dq_cmd.begin());
  return true;
}

void RuckigOptimizer::reset() {
  trajectory_generator_ = new ruckig::Ruckig<capacity_>(1. / *frequency_);
  input_param_ = new ruckig::InputParameter<capacity_>();
  output_param_ = new ruckig::OutputParameter<capacity_>();

  *is_initial_state_set_ = false;
  *is_target_state_set_ = false;

  *start_ = std::chrono::steady_clock::now();
}

}  // namespace rotools
