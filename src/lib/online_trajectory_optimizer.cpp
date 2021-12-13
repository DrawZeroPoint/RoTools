//
// Created by smart on 2021/9/6.
//

#include "roport/online_trajectory_optimizer.h"


namespace rotools {

RuckigOptimizer::RuckigOptimizer(int dof, const std::vector<double>& max_vel, const std::vector<double>& max_acc,
                                 const std::vector<double>& max_jerk, double frequency) {
  if (dof > 7) {
    ROS_WARN("DOF %d exceed the capacity 7", dof);
  }
  dof_ = new int(dof);
  initialized_ = new bool(false);

  trajectory_generator_ = new ruckig::Ruckig<7>(1. / frequency);
  input_param_ = new ruckig::InputParameter<7>();
  output_param_ = new ruckig::OutputParameter<7>();

  if (!max_vel.empty() && !max_acc.empty() && !max_jerk.empty()) {
    for (size_t i = 0; i < dof; i += 1) {
      input_param_->max_velocity[i] = 0.001 * max_vel[i];
      input_param_->max_acceleration[i] = 0.001 * max_acc[i];
      input_param_->max_jerk[i] = 0.001 * max_jerk[i];
    }
  }
}

RuckigOptimizer::~RuckigOptimizer() {
  delete dof_;
  delete initialized_;
  delete trajectory_generator_;
  delete input_param_;
  delete output_param_;
}

void RuckigOptimizer::init(const sensor_msgs::JointState& msg, const std::vector<double>& q_d) {
  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::array<double, 7> acceleration{};
  std::copy(msg.position.begin(), msg.position.end(), position.begin());
  std::copy(msg.velocity.begin(), msg.velocity.end(), velocity.begin());
  input_param_->current_position = position;
  input_param_->current_velocity = velocity;
  input_param_->current_acceleration = acceleration;

  std::array<double, 7> q_desired{};
  std::copy(q_d.begin(), q_d.end(), q_desired.begin());
  input_param_->target_position = q_desired;

  start_ = std::chrono::steady_clock::now();
  *initialized_ = true;
}

bool RuckigOptimizer::set(const std::vector<double>& joint_position, const std::vector<double>& joint_velocity) {
  if (!*initialized_) return false;

  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::copy(joint_position.begin(), joint_position.end(), position.begin());
  std::copy(joint_velocity.begin(), joint_velocity.end(), velocity.begin());
  input_param_->target_position = position;
  input_param_->target_velocity = velocity;
  return true;
}

void RuckigOptimizer::update(std::vector<double>& q_cmd, std::vector<double>& dq_cmd) {
  auto interval = std::chrono::steady_clock::now() - start_;
  auto i_ms = std::chrono::duration_cast<std::chrono::milliseconds>(interval);
  const unsigned long steps = std::max<unsigned long>(i_ms.count(), 1);
  for (unsigned long i = 0; i < steps; i++) {
    trajectory_generator_->update(*input_param_, *output_param_);

    input_param_->current_position = output_param_->new_position;
    input_param_->current_velocity = output_param_->new_velocity;
    input_param_->current_acceleration = output_param_->new_acceleration;
  }
  q_cmd.resize(*dof_);
  dq_cmd.resize(*dof_);
  std::array<double, 7> new_position = output_param_->new_position;
  std::array<double, 7> new_velocity = output_param_->new_velocity;
  std::copy(new_position.begin(), new_position.end(), q_cmd.begin());
  std::copy(new_velocity.begin(), new_velocity.end(), dq_cmd.begin());
}

}  // namespace rotools
