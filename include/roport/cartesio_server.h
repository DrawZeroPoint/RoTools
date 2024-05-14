/*
 * cartesio_server
 * Copyright (c) 2021-2022, Zhipeng Dong
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

#ifndef CARTESIO_SERVER_H
#define CARTESIO_SERVER_H

#include <ros/ros.h>

#include <cartesian_interface/ReachPoseActionGoal.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <cartesian_interface/ReachPoseAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <roport/ExecuteAllLockedPoses.h>
#include <roport/ExecuteAllPoses.h>
#include <roport/ExecuteMirroredPose.h>
#include <roport/ExecuteGroupPose.h>

namespace roport {
class CartesIOServer {
 public:
  CartesIOServer(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~CartesIOServer() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::vector<std::string> group_names_;
  std::vector<std::string> controlled_frames_;
  std::vector<std::string> reference_frames_;

  std::vector<geometry_msgs::Pose> homing_poses_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::ServiceServer execute_group_homing_srv_;

  ros::ServiceServer execute_all_poses_srv_;
  ros::ServiceServer execute_all_locked_poses_srv_;

  using reachPoseActionClient = actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction>;
  std::vector<std::shared_ptr<reachPoseActionClient>> control_clients_;

  /**
   * Move the groups to corresponding poses.
   * @param req
   * @param resp
   * @return
   */
  auto executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req, roport::ExecuteAllPoses::Response& resp) -> bool;

  /**
   * Move the groups to corresponding poses. If only one group is given, this is identical with executeAllPosesSrvCb,
   * if multiple groups are given, the first group's pose will be the target, while other groups will move along with
   * the first and keep their relative poses unchanged during the movement.
   * @param req
   * @param resp
   * @return
   */
  auto executeAllLockedPosesSrvCb(roport::ExecuteAllLockedPoses::Request& req,
                                  roport::ExecuteAllLockedPoses::Response& resp) -> bool;

  auto executeHomingSrvCb(roport::ExecuteGroupPose::Request& req, roport::ExecuteGroupPose::Response& resp) -> bool;

  void buildActionGoal(const int& index,
                       const geometry_msgs::Pose& goal_pose,
                       cartesian_interface::ReachPoseActionGoal& action_goal);

  static void updateStamp(const double& stamp, cartesian_interface::ReachPoseActionGoal& action_goal);

  auto executeGoals(const std::map<int, cartesian_interface::ReachPoseActionGoal>& goals,
                    double duration = 120.0) -> bool;

  bool getTransform(const int& index, geometry_msgs::TransformStamped& transform);

  /**
   * Given the index of the group in group_names_, get current pose of that group's control frame wrt the reference
   * frame.
   * @param index Group index in group_names_.
   * @param pose Pose of the control frame.
   * @return True if succeed, false otherwise.
   */
  bool getCurrentPoseWithIndex(const int& index, geometry_msgs::Pose& pose);

  void getGoalPoseWithReference(const int& ref_idx,
                                const geometry_msgs::Pose& curr_ref_pose,
                                const geometry_msgs::Pose& goal_ref_pose,
                                const int& idx,
                                const geometry_msgs::Pose& curr_pose,
                                geometry_msgs::Pose& goal_pose);
};

}  // namespace roport

#endif  // CARTESIO_SERVER_H
