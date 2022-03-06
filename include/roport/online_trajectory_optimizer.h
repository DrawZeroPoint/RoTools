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
                           double reduce_ratio = 0.001);
  ~RuckigOptimizer();

  void init(const sensor_msgs::JointState& msg, const std::vector<double>& q_d);

  void setInitialState(const sensor_msgs::JointState& msg);

  void setTargetState(const sensor_msgs::JointState& msg);

  /**
   * Set the target position and velocity into the input_param of the optimizer.
   * @param target_position Target joint position.
   * @param target_velocity Target joint velocity.
   * @return True if set, false if the optimizer has not been initialized.
   */
  auto set(const std::vector<double>& target_position, const std::vector<double>& target_velocity) -> bool;

  void update(std::vector<double>& q_cmd, std::vector<double>& dq_cmd);

  auto isInitialStateSet() -> bool { return *is_initial_state_set_; }

  auto isTargetStateSet() -> bool { return *is_target_state_set_; }

  void getTargetPosition(std::vector<double> & q_d);

 private:
  bool* initialized_;
  bool* is_initial_state_set_;
  bool* is_target_state_set_;

  int* dof_;
  std::chrono::steady_clock::time_point* start_;

  ruckig::Ruckig<32>* trajectory_generator_;
  ruckig::InputParameter<32>* input_param_;
  ruckig::OutputParameter<32>* output_param_;
};

}  // namespace rotools

#endif  // SRC_ONLINE_TRAJECTORY_OPTIMIZER_H
