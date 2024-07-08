/*
 * trajectory_planner
 * Copyright (c) 2021-2024, Zhipeng Dong
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>

#ifdef WITH_DRAKE
#include <drake/common/trajectories/piecewise_pose.h>
#endif

#include "roport/common.h"
#include "roport/ExecuteAllCartesianTrajectories.h"
#include "roport/ExecuteGroupPose.h"
#include "roport/GetGroupPose.h"

namespace roport {

class CartesianTrajectoryPlanner {
 public:
  CartesianTrajectoryPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  ~CartesianTrajectoryPlanner() = default;

  /**
   * Make a dense trajectory (i.e., the time step is exactly 0.001s) given the initial pose and sparse waypoints
   * of the controlled frame using Drake.
   * @param initial_pose Initial pose of the end-effector. It could be different with the starting pose of the
   * trajectory.
   * @param trajectory_points Sparse trajectory waypoints
   * @param goal_trajectory Output dense trajectory.
   */
  bool makeCartesianTrajectoryWithDrake(geometry_msgs::Pose initial_pose,
                                        const std::vector<roport::CartesianTrajectoryPoint>& trajectory_points,
                                        roport::CartesianTrajectory& goal_trajectory);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::vector<std::string> declared_group_names_;
  std::vector<std::string> group_names_;

  ros::ServiceServer execute_all_cartesian_trajectory_srv_;

  ros::Duration wait_for_service_timeout_{5.0};

  std::vector<ros::ServiceClient> get_current_pose_clients_;

  double default_time_step_{0.001};

  static void geometryPoseToDrakeRigidTransform(const geometry_msgs::Pose& p, drake::math::RigidTransformd& t);

  static void drakeRigidTransformToGeometryPose(const drake::math::RigidTransformd& t, geometry_msgs::Pose& p);

  static void drakeRigidTransformToCartesianTrajectoryPoint(const drake::math::RigidTransformd& t,
                                                            const Eigen::Matrix<double, 6, 1>& vel,
                                                            const Eigen::Matrix<double, 6, 1>& acc,
                                                            const double& duration,
                                                            roport::CartesianTrajectoryPoint& p);

  void drakeTrajectoryToCartesianTrajectory(const drake::trajectories::PiecewisePose<double>& drake_trajectory,
                                            roport::CartesianTrajectory& trajectory) const;

  auto executeAllCartesianTrajectoryCb(roport::ExecuteAllCartesianTrajectories::Request& req,
                                       roport::ExecuteAllCartesianTrajectories::Response& resp) -> bool;
};

}  // namespace roport
