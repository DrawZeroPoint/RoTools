//
// Created by dzp on 2020/10/14.
//
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

#include <roport/ExecuteAllPoses.h>
#include <roport/ExecuteMirroredPose.h>

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

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::ServiceServer execute_all_poses_srv_;
  ros::ServiceServer execute_mirrored_pose_srv_;

  using reachPoseActionClient = actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction>;
  std::vector<std::shared_ptr<reachPoseActionClient>> control_clients_;

  auto executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req, roport::ExecuteAllPoses::Response& resp) -> bool;
  //  auto executeMirroredPoseSrvCb(roport::ExecuteMirroredPose::Request& req, roport::ExecuteMirroredPose::Response&
  //  resp)
  //      -> bool;
  //

  void buildActionGoal(const int& index,
                       const geometry_msgs::Pose& goal_pose,
                       cartesian_interface::ReachPoseActionGoal& action_goal);

  static void updateStamp(const double& stamp, cartesian_interface::ReachPoseActionGoal& action_goal);

  auto executeGoals(const std::map<int, cartesian_interface::ReachPoseActionGoal>& goals, double duration = 120)
      -> bool;

  //  static void getMirroredTrajectory(MoveGroupInterface& move_group,
  //                                    trajectory_msgs::JointTrajectory trajectory,
  //                                    std::vector<double> mirror_vector,
  //                                    trajectory_msgs::JointTrajectory& mirrored_trajectory);

  bool getTransform(const int& index, geometry_msgs::TransformStamped& transform);
};

}  // namespace roport

#endif  // CARTESIO_SERVER_H
