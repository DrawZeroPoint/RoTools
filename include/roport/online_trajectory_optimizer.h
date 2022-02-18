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
                           double frequency = 1000.);
  ~RuckigOptimizer();

  void init(const sensor_msgs::JointState& msg, const std::vector<double>& q_d);

  auto set(const std::vector<double>& joint_position, const std::vector<double>& joint_velocity) -> bool;

  void update(std::vector<double>& q_cmd, std::vector<double>& dq_cmd);

  auto isInitialized() -> bool { return *initialized_; }

 private:
  bool* initialized_;

  int* dof_;
  std::chrono::steady_clock::time_point start_;

  ruckig::Ruckig<7>* trajectory_generator_;
  ruckig::InputParameter<7>* input_param_;
  ruckig::OutputParameter<7>* output_param_;
};

}  // namespace rotools

#endif  // SRC_ONLINE_TRAJECTORY_OPTIMIZER_H
