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
#include <ros/ros.h>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#ifdef WITH_DRAKE
#include <drake/common/find_resource.h>
#include <drake/common/text_logging.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_pose.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree.h>
#include <drake/planning/trajectory_optimization/direct_collocation.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#endif

#include "roport/common.h"
#include "roport/ExecuteAllCartesianTrajectories.h"
#include "roport/ExecuteGroupCartesianTrajectory.h"
#include "roport/GetGroupPose.h"

namespace roport {

using namespace drake::planning::trajectory_optimization;

class TrajectoryPlanner {
 public:
  TrajectoryPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  ~TrajectoryPlanner() = default;

  /**
   * Make a dense trajectory (i.e., the time step is exactly 0.001s) given the initial pose and sparse waypoints
   * of the controlled frame using Drake.
   * @param initial_pose Initial pose of the end-effector. It could be different with the starting pose of the
   * trajectory.
   * @param sparse_trajectory Sparse trajectory.
   * @param dense_trajectory Output dense trajectory.
   */
  bool makeCartesianTrajectoryWithDrake(geometry_msgs::Pose initial_pose,
                                        const roport::CartesianTrajectory& sparse_trajectory,
                                        roport::CartesianTrajectory& dense_trajectory);

  bool makeJointTrajectoryWithDrake(const roport::CartesianTrajectory& sparse_trajectory,
                                    trajectory_msgs::JointTrajectory& joint_trajectory);

  bool makeJointTrajectoryWithDrake(const roport::ExecuteAllCartesianTrajectories::Request& request,
                                    trajectory_msgs::JointTrajectory& joint_trajectory);

  void displayCartesianTrajectoryInRViz(const int& index,
                                        const roport::CartesianTrajectory& cartesian_trajectory,
                                        const int& step = 100);

  void displayJointTrajectoryInRViz(const trajectory_msgs::JointTrajectory& joint_trajectory);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::vector<std::string> group_names_;

  bool visualize_;
  ros::Publisher joint_trajectory_publisher_;
  std::vector<ros::Publisher> cartesian_trajectory_publishers_;

  ros::Subscriber joint_state_subscriber_;
  sensor_msgs::JointState current_joint_state_;
  std::vector<std::vector<std::string>> group_joint_names_;

  bool is_execute_;
  ros::ServiceServer execute_all_cartesian_trajectory_srv_;
  ros::ServiceServer execute_joint_trajectory_with_cartesian_trajectories_srv_;

  ros::Duration wait_for_service_timeout_{1.0};

  std::vector<ros::ServiceClient> get_current_pose_clients_;
  std::vector<ros::ServiceClient> execute_group_cartesian_trajectory_clients_;

  // Drake objects
  drake::multibody::MultibodyPlant<double> plant_;
  drake::multibody::Parser parser_;
  std::vector<drake::multibody::ModelInstanceIndex> model_indexes_;

  double default_time_step_{0.001};
  double joint_traj_time_step_{0.1};

  static void geometryPoseToRigidTransform(const geometry_msgs::Pose& p, drake::math::RigidTransformd& t);

  void jointTrajectoryPointToDrakePosition(const std::vector<std::string>& joint_names,
                                           const trajectory_msgs::JointTrajectoryPoint& wp,
                                           Eigen::VectorXd& q);

  static void rigidTransformToGeometryPose(const drake::math::RigidTransformd& t, geometry_msgs::Pose& p);

  static void rigidTransformToCartesianTrajectoryPoint(const drake::math::RigidTransformd& pose,
                                                       const Eigen::Matrix<double, 6, 1>& vel,
                                                       const Eigen::Matrix<double, 6, 1>& acc,
                                                       const double& duration,
                                                       roport::CartesianTrajectoryPoint& p);

  void drakeTrajectoryToCartesianTrajectory(const drake::trajectories::PiecewisePose<double>& drake_trajectory,
                                            roport::CartesianTrajectory& trajectory) const;

  void drakePositionToJointTrajectoryPoint(const Eigen::VectorXd& q,
                                           const double& time_from_start,
                                           trajectory_msgs::JointTrajectoryPoint& wp);

  auto executeAllCartesianTrajectoriesCb(roport::ExecuteAllCartesianTrajectories::Request& req,
                                         roport::ExecuteAllCartesianTrajectories::Response& resp) -> bool;

  auto executeJointTrajectoryWithCartesianTrajectoriesCb(roport::ExecuteAllCartesianTrajectories::Request& req,
                                                         roport::ExecuteAllCartesianTrajectories::Response& resp)
      -> bool;

  void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg);

  void currentJointStatesToDrakePosition(Eigen::VectorXd& q);

  bool generateJointTrajectoryWithIK(const std::vector<roport::CartesianTrajectory>& c_trajectories,
                                     trajectory_msgs::JointTrajectory& sparse_j_trajectory);

  bool generateDenseJointTrajectoryWithOptimization(const trajectory_msgs::JointTrajectory& sparse_joint_trajectory,
                                                    trajectory_msgs::JointTrajectory& dense_joint_trajectory);

  void generateDenseJointTrajectory(const trajectory_msgs::JointTrajectory& sparse_joint_trajectory,
                                    trajectory_msgs::JointTrajectory& dense_joint_trajectory);

  /** This function is under dev **/
  bool optimizeDrakeJointTrajectory(const drake::trajectories::PiecewisePolynomial<double>& traj,
                                    drake::trajectories::PiecewisePolynomial<double>& traj_opt);

  /** This function is under dev **/
  bool optimizePiecewisePolynomial(const drake::trajectories::PiecewisePolynomial<double>& traj,
                                   drake::trajectories::PiecewisePolynomial<double>& traj_opt);
};

}  // namespace roport
