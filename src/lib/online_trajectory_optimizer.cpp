//
// Created by smart on 2021/9/6.
//

#include "roport/online_trajectory_optimizer.h"

using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

namespace rotools {

OnlineTrajectoryOptimizer::OnlineTrajectoryOptimizer(int dof, double frequency)
    : dof_(dof), frequency_(frequency), enabled_(false), initialized_(false) {
  reflex_ = new ReflexxesAPI(dof_, 1.0 / frequency_);
  input_param_ = new RMLPositionInputParameters(dof_);
  output_param_ = new RMLPositionOutputParameters(dof_);
  flags_.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
}

OnlineTrajectoryOptimizer::~OnlineTrajectoryOptimizer() {
  delete reflex_;
  delete input_param_;
  delete output_param_;
}

void OnlineTrajectoryOptimizer::setParameters(const std::vector<double>& max_vel, const std::vector<double>& max_acc,
                                              const std::vector<double>& max_jerk, double vel_scale, double acc_scale,
                                              double jerk_scale) {
  if (dof_ != max_vel.size()) {
    ROS_ERROR("ERROR: DOF mismatch! Defined dof: [%zu] Actual dof: [%zu]", dof_, max_vel.size());
    return;
  }
  for (int i = 0; i < dof_; i++) {
    input_param_->MaxVelocityVector->VecData[i] = max_vel[i] * vel_scale;
    input_param_->MaxAccelerationVector->VecData[i] = max_acc[i] * acc_scale;
    input_param_->MaxJerkVector->VecData[i] = max_jerk[i] * jerk_scale;
    input_param_->SelectionVector->VecData[i] = true;
    ROS_WARN_STREAM("OTO params for dof " << i << ": max_vel: " << input_param_->MaxVelocityVector->VecData[i]
                                          << ", max_acc: " << input_param_->MaxAccelerationVector->VecData[i]
                                          << ", max_jerk: " << input_param_->MaxJerkVector->VecData[i]);
  }
  enabled_ = true;
}

void OnlineTrajectoryOptimizer::update(const std::vector<double>& q, const std::vector<double>& dq) {
  for (size_t i = 0; i < dof_; i++) {
    input_param_->CurrentPositionVector->VecData[i] = q[i];
    input_param_->CurrentVelocityVector->VecData[i] = dq[i];
    input_param_->CurrentAccelerationVector->VecData[i] = 0;
  }
  initialized_ = true;
}

bool OnlineTrajectoryOptimizer::output(const std::vector<double>& q_desired, const std::vector<double>& dq_desired,
                                       std::vector<double>& q_out, std::vector<double>& dq_out, std::vector<double>& ddq_out) {
  if (!enabled_ || !initialized_) return false;

  for (size_t i = 0; i < dof_; i++) {
    input_param_->TargetPositionVector->VecData[i] = q_desired[i];
    input_param_->TargetVelocityVector->VecData[i] = dq_desired[i];
  }

  int result_value = reflex_->RMLPosition(*input_param_, output_param_, flags_);
  if (result_value < 0) {
    ROS_ERROR("Error occurred in Reflexxes: %d", result_value);
    return false;
  }

  input_param_->CurrentPositionVector = output_param_->NewPositionVector;
  input_param_->CurrentVelocityVector = output_param_->NewVelocityVector;
  input_param_->CurrentAccelerationVector = output_param_->NewAccelerationVector;

  for (size_t i = 0; i < dof_; i++) {
    q_out.push_back(output_param_->NewPositionVector->VecData[i]);
    dq_out.push_back(output_param_->NewVelocityVector->VecData[i]);
    ddq_out.push_back(output_param_->NewAccelerationVector->VecData[i]);
  }
  return true;
}

RuckigOptimizer::RuckigOptimizer(int dof, const std::vector<double>& max_vel, const std::vector<double>& max_acc,
                                 const std::vector<double>& max_jerk, double frequency)
    : dof_(dof), initialized_(false) {
  if (dof > 7) {
    ROS_WARN("DOF %d exceed the capacity 7", dof);
  }

  trajectory_generator_ = new ruckig::Ruckig<7>(1. / frequency);
  input_param_ = new ruckig::InputParameter<7>();
  output_param_ = new ruckig::OutputParameter<7>();

  for (size_t i = 0; i < dof; i += 1) {
    input_param_->max_velocity[i] = 0.1 * max_vel[i];
    input_param_->max_acceleration[i] = 0.1 * max_acc[i];
    input_param_->max_jerk[i] = 0.1 * max_jerk[i];
  }
}

RuckigOptimizer::~RuckigOptimizer() {
  delete trajectory_generator_;
  delete input_param_;
  delete output_param_;
}

void RuckigOptimizer::init(const sensor_msgs::JointState& msg, const std::vector<double>& q_d) {
  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::array<double, 7> acceleration{};
  std::copy(position.begin(), position.end(), msg.position.begin());
  std::copy(velocity.begin(), velocity.end(), msg.velocity.begin());
  input_param_->current_position = position;
  input_param_->current_velocity = velocity;
  input_param_->current_acceleration = acceleration;

  std::array<double, 7> q_desired{};
  std::copy(q_desired.begin(), q_desired.end(), q_d.begin());
  input_param_->target_position = q_desired;

  start_ = std::chrono::steady_clock::now();
  initialized_ = true;
}

void RuckigOptimizer::set(const std::vector<double>& joint_position, const std::vector<double>& joint_velocity) {
  if (!initialized_) return;

  std::array<double, 7> position{};
  std::array<double, 7> velocity{};
  std::copy(position.begin(), position.end(), joint_position.begin());
  std::copy(velocity.begin(), velocity.end(), joint_velocity.begin());
  input_param_->target_position = position;
  input_param_->target_velocity = velocity;
}

void RuckigOptimizer::update(std::vector<double>& q_cmd) {
  auto interval = std::chrono::steady_clock::now() - start_;
  auto i_ms = std::chrono::duration_cast<std::chrono::milliseconds>(interval);
  const unsigned long steps = std::max<unsigned long>(i_ms.count(), 1);
  for (unsigned long i = 0; i < steps; i++) {
    trajectory_generator_->update(*input_param_, *output_param_);

    input_param_->current_position = output_param_->new_position;
    input_param_->current_velocity = output_param_->new_velocity;
    input_param_->current_acceleration = output_param_->new_acceleration;
  }
  q_cmd.resize(dof_);
  std::array<double, 7> new_position = output_param_->new_position;
  std::copy(new_position.begin(), new_position.end(), q_cmd);
}

}  // namespace rotools
