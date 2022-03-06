//
// Created by smart on 2021/9/6.
//

#include "roport/online_trajectory_optimizer.h"

namespace rotools {

RuckigOptimizer::RuckigOptimizer(int dof,
                                 const std::vector<double>& max_vel,
                                 const std::vector<double>& max_acc,
                                 const std::vector<double>& max_jerk,
                                 double frequency,
                                 double reduce_ratio) {
  const int capacity = 32;  // This value can be raised if needed
  if (dof > capacity) {
    ROS_ERROR("RoTools: DOF %d exceed the capacity (%d) of the online trajectory optimizer", dof, capacity);
    return;
  }
  assert(reduce_ratio > 0 && frequency > 0);
  assert(!max_vel.empty() && max_vel.size() == max_acc.size() && max_vel.size() == max_jerk.size());

  dof_ = new int(dof);
  initialized_ = new bool(false);
  is_initial_state_set_ = new bool(false);
  is_target_state_set_ = new bool(false);
  start_ = new std::chrono::steady_clock::time_point;

  trajectory_generator_ = new ruckig::Ruckig<capacity>(1. / frequency);
  input_param_ = new ruckig::InputParameter<capacity>();
  output_param_ = new ruckig::OutputParameter<capacity>();

  for (size_t i = 0; i < capacity; i += 1) {
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
  delete dof_;
  delete initialized_;
  delete trajectory_generator_;
  delete input_param_;
  delete output_param_;
  delete start_;
}

void RuckigOptimizer::init(const sensor_msgs::JointState& msg, const std::vector<double>& q_d) {
  std::array<double, 32> position{};
  std::array<double, 32> velocity{};
  std::array<double, 32> acceleration{};
  std::copy(msg.position.begin(), msg.position.end(), position.begin());
  std::copy(msg.velocity.begin(), msg.velocity.end(), velocity.begin());
  input_param_->current_position = position;
  input_param_->current_velocity = velocity;
  input_param_->current_acceleration = acceleration;

  std::array<double, 32> q_desired{};
  std::copy(q_d.begin(), q_d.end(), q_desired.begin());
  input_param_->target_position = q_desired;

  *start_ = std::chrono::steady_clock::now();
  *initialized_ = true;
}

void RuckigOptimizer::setInitialState(const sensor_msgs::JointState& msg) {
  std::array<double, 32> position{};
  std::array<double, 32> velocity{};
  std::array<double, 32> acceleration{};
  std::copy(msg.position.begin(), msg.position.end(), position.begin());
  std::copy(msg.velocity.begin(), msg.velocity.end(), velocity.begin());
  input_param_->current_position = position;
  input_param_->current_velocity = velocity;
  input_param_->current_acceleration = acceleration;
  *is_initial_state_set_ = true;
  *start_ = std::chrono::steady_clock::now();
}

void RuckigOptimizer::setTargetState(const sensor_msgs::JointState& msg) {
  std::array<double, 32> position{};
  std::array<double, 32> velocity{};
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

bool RuckigOptimizer::set(const std::vector<double>& target_position, const std::vector<double>& target_velocity) {
  if (!*initialized_) {
    return false;
  }

  std::array<double, 32> position{};
  std::array<double, 32> velocity{};
  std::copy(target_position.begin(), target_position.end(), position.begin());
  std::copy(target_velocity.begin(), target_velocity.end(), velocity.begin());
  input_param_->target_position = position;
  input_param_->target_velocity = velocity;
  return true;
}

void RuckigOptimizer::update(std::vector<double>& q_cmd, std::vector<double>& dq_cmd) {
  assert(*is_initial_state_set_ && *is_target_state_set_);

  auto interval = std::chrono::steady_clock::now() - *start_;
  auto i_ms = std::chrono::duration_cast<std::chrono::milliseconds>(interval);
  const unsigned long steps = std::max<unsigned long>(i_ms.count(), 1);
  for (unsigned long i = 0; i < steps; i++) {
    trajectory_generator_->update(*input_param_, *output_param_);

    input_param_->current_position = output_param_->new_position;
    input_param_->current_velocity = output_param_->new_velocity;
    input_param_->current_acceleration = output_param_->new_acceleration;
  }
  std::array<double, 32> new_position = output_param_->new_position;
  std::array<double, 32> new_velocity = output_param_->new_velocity;

  q_cmd.resize(*dof_);
  dq_cmd.resize(*dof_);
  std::copy(new_position.begin(), new_position.begin() + *dof_, q_cmd.begin());
  std::copy(new_velocity.begin(), new_velocity.begin() + *dof_, dq_cmd.begin());
}

}  // namespace rotools
