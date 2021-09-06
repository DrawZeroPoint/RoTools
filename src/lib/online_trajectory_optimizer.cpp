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
      : dof_(dof), frequency_(frequency), enabled_(false), initialized_(false)
  {
    reflex_ = new ReflexxesAPI(dof_, 1.0 / frequency_);
    input_param_ = new RMLPositionInputParameters(dof_);
    output_param_ = new RMLPositionOutputParameters(dof_);
    flags_.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
  }

  OnlineTrajectoryOptimizer::~OnlineTrajectoryOptimizer()
  {
    delete reflex_;
    delete input_param_;
    delete output_param_;
  }

  void OnlineTrajectoryOptimizer::setParameters(const std::vector<double>& max_vel, const std::vector<double>& max_acc,
                                                const std::vector<double>& max_jerk,
                                                double vel_scale, double acc_scale, double jerk_scale)
  {
    if (dof_ != max_vel.size()) {
      ROS_ERROR("ERROR: DOF mismatch! Defined dof: [%zu] Actual dof: [%zu]", dof_, max_vel.size());
      return;
    }
    for (int i = 0; i < dof_; i++) {
      input_param_->MaxVelocityVector->VecData[i] = max_vel[i] * vel_scale;
      input_param_->MaxAccelerationVector->VecData[i] = max_acc[i] * acc_scale;
      input_param_->MaxJerkVector->VecData[i] = max_jerk[i] * jerk_scale;
      input_param_->SelectionVector->VecData[i] = true;
      ROS_WARN_STREAM("OTO params for dof "
                          << i <<
                          ": max_vel: "
                          << input_param_->MaxVelocityVector->VecData[i] <<
                          ", max_acc: "
                          << input_param_->MaxAccelerationVector->VecData[i] <<
                          ", max_jerk: "
                          << input_param_->MaxJerkVector->VecData[i]
      );
    }
    enabled_ = true;
  }

  void OnlineTrajectoryOptimizer::update(const std::vector<double>& q, const std::vector<double>& dq)
  {
    for (size_t i = 0; i < dof_; i++) {
      input_param_->CurrentPositionVector->VecData[i] = q[i];
      input_param_->CurrentVelocityVector->VecData[i] = dq[i];
      input_param_->CurrentAccelerationVector->VecData[i] = 0;
    }
    initialized_ = true;
  }

  bool OnlineTrajectoryOptimizer::output(const std::vector<double>& q_desired, const std::vector<double>& dq_desired,
                                         std::vector<double> &q_out, std::vector<double> &dq_out,
                                         std::vector<double> &ddq_out)
  {
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
}

