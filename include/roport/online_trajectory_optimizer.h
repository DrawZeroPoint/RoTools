//
// Created by smart on 2021/9/6.
//

#ifndef SRC_ONLINE_TRAJECTORY_OPTIMIZER_H
#define SRC_ONLINE_TRAJECTORY_OPTIMIZER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "ruckig/ruckig.hpp"

namespace rotools {

class RuckigOptimizer {
 public:
  RuckigOptimizer() = delete;
  explicit RuckigOptimizer(int dof,
                           const std::vector<double>& max_vel,
                           const std::vector<double>& max_acc,
                           const std::vector<double>& max_jerk,
                           double frequency = 1000.,
                           double reduce_ratio = 0.005);
  ~RuckigOptimizer();

  /**
   * Set the initial position and velocity into the input_param.
   * @param msg Measured msg containing the current state.
   * @note The call of setInitialState should happen after setTargetState,
   * since the timer is started here.
   */
  void setInitialState(const sensor_msgs::JointState& msg);

  /**
   * Set the target position and velocity into the input_param of the optimizer.
   * @param msg Control msg containing target position and velocity.
   */
  void setTargetState(const sensor_msgs::JointState& msg);

  /**
   * With the internally recorded execution time, estimate the current position and velocity the robot
   * can reach under the constraints and set the estimations as commands to the robot.
   * @param q_cmd Output joint position command.
   * @param dq_cmd Output joint velocity command.
   */
  void update(std::vector<double>& q_cmd, std::vector<double>& dq_cmd);

  void reset();

  auto isInitialStateSet() -> bool { return *is_initial_state_set_; }

  auto isTargetStateSet() -> bool { return *is_target_state_set_; }

  /**
   * Get the target position stored in the input_param.
   * @param q_d Output target position.
   */
  void getTargetPosition(std::vector<double>& q_d);

 private:
  static constexpr int capacity_ = 32;

  bool* is_initial_state_set_;
  bool* is_target_state_set_;

  int* dof_;
  double* frequency_;
  std::chrono::steady_clock::time_point* start_;

  ruckig::Ruckig<capacity_>* trajectory_generator_;
  ruckig::InputParameter<capacity_>* input_param_;
  ruckig::OutputParameter<capacity_>* output_param_;
};

}  // namespace rotools

#endif  // SRC_ONLINE_TRAJECTORY_OPTIMIZER_H
