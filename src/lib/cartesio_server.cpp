//
// Created by dzp on 2020/10/14.
//

#include "roport/cartesio_server.h"

#include <utility>

namespace roport {

CartesIOServer::CartesIOServer(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh) {
  // Get all available planning group names
  XmlRpc::XmlRpcValue group_names;
  pnh_.getParam("group_names", group_names);
  ROS_ASSERT(group_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (group_names.size() == 0) {
    throw std::runtime_error("RoPort: Move group name list is None");
  }
  for (int i = 0; i < group_names.size(); i++) {
    ROS_ASSERT(group_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    group_names_.push_back(group_names[i]);
  }

  // Get global reference frame for all groups
  pnh_.getParam("reference_frame", reference_frame_);

  // Initialize planning groups
  //  for (auto& group_name : group_names_) {
  //    ROS_INFO_STREAM("RoPort: Initializing move group interface " << group_name);
  //    auto group = std::make_shared<MoveGroupInterface>(group_name);
  //    if (!reference_frame.empty()) {
  //      group->setPoseReferenceFrame(reference_frame);
  //    }
  //    interfaces_.push_back(group);
  //  }

  // Initialize controller action clients, one for each group
  const double kTimeout = 3.;
  for (auto& group_name : group_names_) {
    std::string action_name = "cartesian/" + group_name + "/reach";
    auto client = std::make_shared<reachPoseActionClient>(action_name);
    if (!client->waitForServer(ros::Duration(kTimeout))) {
      throw std::runtime_error("RoPort: Action server " + action_name + " unavailable");
    }
    control_clients_.push_back(client);
    ROS_INFO_STREAM("RoPort: Controller action client " << action_name << " initialized");
  }

  // Initialize control servers
//  execute_all_poses_srv_ = nh_.advertiseService("execute_all_poses", &CartesIOServer::executeAllPosesSrvCb, this);
  //  execute_mirrored_pose_srv_ =
  //      nh_.advertiseService("execute_mirrored_pose", &CartesIOServer::executeMirroredPoseSrvCb, this);
}

//auto CartesIOServer::executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req,
//                                          roport::ExecuteAllPoses::Response& resp) -> bool {
//  ROS_ASSERT(req.group_names.size() == req.goals.poses.size());
//  std::map<int, trajectory_msgs::JointTrajectory> trajectories;
//
//  for (int i = 0; i < group_names_.size(); ++i) {
//    for (int j = 0; j < req.group_names.size(); ++j) {
//      if (req.group_names[j] == group_names_[i]) {
//        trajectory_msgs::JointTrajectory trajectory;
//        geometry_msgs::PoseStamped goal_pose_wrt_base;
//        goal_pose_wrt_base.header.frame_id = reference_frame_;
//
//        // Transfer given pose to base frame
//        if (req.goal_type == req.BASE_ABS) {
//          goal_pose_wrt_base.pose = req.goals.poses[j];
//        } else if (req.goal_type == req.BASE_REL) {
//          geometry_msgs::PoseStamped transform_wrt_local_base;
//          transform_wrt_local_base.pose = req.goals.poses[j];
//          relativePoseToAbsolutePose(transform_wrt_local_base, current_pose_wrt_base, goal_pose_wrt_base);
//        } else {
//          geometry_msgs::PoseStamped goal_pose_wrt_eef;
//          goal_pose_wrt_eef.pose = req.goals.poses[j];
//          goal_pose_wrt_eef.header.frame_id = interfaces_[i]->getEndEffectorLink();
//          goal_pose_wrt_base =
//              tf_buffer_.transform(goal_pose_wrt_eef, interfaces_[i]->getPlanningFrame(), ros::Duration(1));
//        }
//
//        // Build trajectory to reach the goal
//        if (!buildTrajectory(*interfaces_[i], goal_pose_wrt_base, trajectory)) {
//          ROS_ERROR("Building trajectory failed");
//          resp.result_status = resp.FAILED;
//          return true;
//        }
//        if (req.stamps.size() == req.group_names.size() && req.stamps[j] > 0) {
//          trajectory_msgs::JointTrajectory updated_trajectory;
//          updateTrajectoryStamp(trajectory, req.stamps[j], updated_trajectory);
//          trajectories.insert({i, updated_trajectory});
//        } else {
//          trajectories.insert({i, trajectory});
//        }
//      }
//    }
//  }
//  if (executeTrajectories(trajectories)) {
//    resp.result_status = resp.SUCCEEDED;
//  } else {
//    resp.result_status = resp.FAILED;
//  }
//  return true;
//}

// auto CartesIOServer::executeMirroredPoseSrvCb(roport::ExecuteMirroredPose::Request& req,
//                                               roport::ExecuteMirroredPose::Response& resp) -> bool {
//   int ref_index = getGroupIndex(req.reference_group);
//   if (ref_index < 0) {
//     throw std::runtime_error("Reference group does not exist");
//   }
//   int mirror_index = getGroupIndex(req.mirror_group);
//   if (mirror_index < 0) {
//     throw std::runtime_error("Mirror group does not exist");
//   }
//
//   std::map<int, trajectory_msgs::JointTrajectory> trajectories;
//   trajectory_msgs::JointTrajectory trajectory;
//   geometry_msgs::PoseStamped current_pose_wrt_base = interfaces_[ref_index]->getCurrentPose();
//   geometry_msgs::PoseStamped goal_pose_wrt_base;
//   goal_pose_wrt_base.header = current_pose_wrt_base.header;
//
//   // Transfer given goal pose to the base frame
//   if (req.goal_type == req.BASE_ABS) {
//     goal_pose_wrt_base.pose = req.goal;
//   } else if (req.goal_type == req.BASE_REL) {
//     geometry_msgs::PoseStamped transform_wrt_local_base;
//     transform_wrt_local_base.pose = req.goal;
//     relativePoseToAbsolutePose(transform_wrt_local_base, current_pose_wrt_base, goal_pose_wrt_base);
//   } else {
//     geometry_msgs::PoseStamped goal_pose_wrt_eef;
//     goal_pose_wrt_eef.pose = req.goal;
//     goal_pose_wrt_eef.header.frame_id = interfaces_[ref_index]->getEndEffectorLink();
//     goal_pose_wrt_base =
//         tf_buffer_.transform(goal_pose_wrt_eef, interfaces_[ref_index]->getPlanningFrame(), ros::Duration(1));
//   }
//
//   // Build trajectory to reach the goal
//   if (!buildTrajectory(*interfaces_[ref_index], goal_pose_wrt_base, trajectory, req.is_cartesian)) {
//     ROS_ERROR("RoPort: Building trajectory failed");
//     resp.result_status = resp.FAILED;
//     return true;
//   }
//
//   trajectory_msgs::JointTrajectory updated_trajectory;
//   if (req.stamp > 0) {
//     updateTrajectoryStamp(trajectory, req.stamp, updated_trajectory);
//   } else {
//     updated_trajectory = trajectory;
//   }
//   trajectories.insert({ref_index, updated_trajectory});
//
//   trajectory_msgs::JointTrajectory mirrored_trajectory;
//   getMirroredTrajectory(*interfaces_[mirror_index], updated_trajectory, req.mirror_vector, mirrored_trajectory);
//   trajectories.insert({mirror_index, mirrored_trajectory});
//
//   if (executeTrajectories(trajectories)) {
//     resp.result_status = resp.SUCCEEDED;
//   } else {
//     resp.result_status = resp.FAILED;
//   }
//   return true;
// }
//
// void CartesIOServer::relativePoseToAbsolutePose(const geometry_msgs::PoseStamped& transform_wrt_local_base,
//                                                 const geometry_msgs::PoseStamped& current_pose_wrt_base,
//                                                 geometry_msgs::PoseStamped& goal_pose_wrt_base) {
//   Eigen::Matrix4d mat_be;
//   geometryPoseToEigenMatrix(current_pose_wrt_base, mat_be);
//   // mat_bl
//   Eigen::Matrix4d mat_bl = Eigen::Matrix4d::Identity();
//   Eigen::Vector3d t_bl(current_pose_wrt_base.pose.position.x, current_pose_wrt_base.pose.position.y,
//                        current_pose_wrt_base.pose.position.z);
//   mat_bl.topRightCorner(3, 1) = t_bl;
//   // mat_le
//   Eigen::Matrix4d mat_le = mat_bl.inverse() * mat_be;
//   // mat_ll_new
//   Eigen::Matrix4d mat_ll_new;
//   geometryPoseToEigenMatrix(transform_wrt_local_base, mat_ll_new);
//   // mat_be_new
//   Eigen::Matrix4d mat_be_new = mat_bl * mat_ll_new * mat_le;
//   eigenMatrixToGeometryPose(mat_be_new, goal_pose_wrt_base);
// }
//
// void CartesIOServer::getMirroredTrajectory(MoveGroupInterface& move_group,
//                                            trajectory_msgs::JointTrajectory trajectory,
//                                            std::vector<double> mirror_vector,
//                                            trajectory_msgs::JointTrajectory& mirrored_trajectory) {
//   mirrored_trajectory.joint_names = move_group.getActiveJoints();
//   mirrored_trajectory.header = trajectory.header;
//   for (size_t i = 0; i < trajectory.points.size(); ++i) {
//     trajectory_msgs::JointTrajectoryPoint p;
//     p = trajectory.points[i];
//     ROS_ASSERT(p.positions.size() == mirror_vector.size());
//     for (size_t j = 0; j < p.positions.size(); ++j) {
//       p.positions[j] *= mirror_vector[j];
//       if (!p.velocities.empty()) {
//         p.velocities[j] *= mirror_vector[j];
//       }
//       if (!p.accelerations.empty()) {
//         p.accelerations[j] *= mirror_vector[j];
//       }
//       if (!p.effort.empty()) {
//         p.effort[j] *= mirror_vector[j];
//       }
//     }
//     mirrored_trajectory.points.push_back(p);
//   }
// }
//
// auto CartesIOServer::buildTrajectory(MoveGroupInterface& move_group,
//                                      const geometry_msgs::PoseStamped& pose,
//                                      trajectory_msgs::JointTrajectory& trajectory,
//                                      bool do_cartesian) -> bool {
//   moveit_msgs::RobotTrajectory robot_trajectory;
//   if (!do_cartesian) {
//     move_group.clearPoseTargets();
//     move_group.setPoseTarget(pose);
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto res = move_group.plan(plan);
//     if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS) {  // NOLINT
//       ROS_ERROR_STREAM("RoPort: Planning for group " + move_group.getName() + " failed with MoveItErrorCode " +
//                        std::to_string(res.val));
//       return false;
//     }
//     robot_trajectory = plan.trajectory_;
//   } else {
//     const double kJumpThreshold = 0.0;  // No joint-space jump constraint (see moveit_msgs/GetCartesianPath)
//     const double kEEStep = 0.01;
//
//     std::vector<geometry_msgs::Pose> way_points;
//     way_points.push_back(pose.pose);
//     double fraction = move_group.computeCartesianPath(way_points, kEEStep, kJumpThreshold, robot_trajectory);
//     if (fraction != 1) {
//       ROS_ERROR_STREAM("RoPort: Planning for group " + move_group.getName() + " failed with fraction " +
//                        std::to_string(fraction));
//       return false;
//     }
//   }
//   trajectory = robot_trajectory.joint_trajectory;
//   trajectory.joint_names = move_group.getActiveJoints();
//   assert(!trajectory.joint_names.empty());
//   return true;
// }
//
// auto CartesIOServer::buildTrajectory(const std::shared_ptr<MoveGroupInterface>& move_group,
//                                      const std::vector<geometry_msgs::PoseStamped>& poses,
//                                      const std::vector<ros::Duration>& time_stamps,
//                                      trajectory_msgs::JointTrajectory& trajectory) -> bool {
//   if (poses.empty()) {
//     throw std::invalid_argument("RoPort: Received empty pose vector");
//   }
//
//   if (poses.size() != time_stamps.size()) {
//     throw std::invalid_argument("RoPort: Poses and time_stamps should have the same size, but " +
//                                 std::to_string(poses.size()) + " vs " + std::to_string(time_stamps.size()));
//   }
//
//   moveit::core::RobotStatePtr robot_state = move_group->getCurrentState();
//   std::string group_name = move_group->getName();
//   const moveit::core::JointModelGroup* joint_group = move_group->getCurrentState()->getJointModelGroup(group_name);
//
//   std::vector<std::vector<double>> joint_poses;
//   for (unsigned int i = 0; i < poses.size(); i++) {
//     const geometry_msgs::Pose& pose = poses[i].pose;
//     std::vector<double> joint_pose;
//     if (!robot_state->setFromIK(joint_group, pose, move_group->getEndEffectorLink(), 1.0)) {
//       throw std::runtime_error("RoPort: followEePoseTrajectory failed, IK solution not found for pose " +
//                                std::to_string(i) + " ( requested pose is (" + std::to_string(pose.position.x) + ", "
//                                + std::to_string(pose.position.y) + ", " + std::to_string(pose.position.z) + ") (" +
//                                std::to_string(pose.orientation.x) + ", " + std::to_string(pose.orientation.y) + ", "
//                                + std::to_string(pose.orientation.z) + ", " + std::to_string(pose.orientation.w) +
//                                ") transformed from " + poses.at(i).header.frame_id + ")");
//     }
//     robot_state->copyJointGroupPositions(joint_group, joint_pose);
//     joint_poses.push_back(joint_pose);
//   }
//
//   trajectory.joint_names = joint_group->getActiveJointModelNames();
//   for (unsigned int i = 0; i < joint_poses.size(); i++) {
//     trajectory_msgs::JointTrajectoryPoint point;
//     point.positions = joint_poses.at(i);
//     point.time_from_start = time_stamps.at(i);
//     trajectory.points.push_back(point);
//   }
//   return true;
// }
//
// void CartesIOServer::updateTrajectoryStamp(trajectory_msgs::JointTrajectory traj_in,
//                                            double stamp,
//                                            trajectory_msgs::JointTrajectory& traj_out) {
//   traj_out.header = traj_in.header;
//   traj_out.joint_names = traj_in.joint_names;
//
//   ros::Duration last_stamp = traj_in.points[traj_in.points.size() - 1].time_from_start;
//   double ratio = stamp / last_stamp.toSec();
//   for (auto& traj_point : traj_in.points) {
//     trajectory_msgs::JointTrajectoryPoint point;
//     point.positions = traj_point.positions;
//     point.time_from_start = ros::Duration(traj_point.time_from_start.toSec() * ratio);
//     traj_out.points.push_back(point);
//   }
// }
//
// auto CartesIOServer::executeTrajectories(const std::map<int, trajectory_msgs::JointTrajectory>& trajectories,
//                                          double duration) -> bool {
//   std::vector<control_msgs::FollowJointTrajectoryGoal> goals;
//   for (const auto& traj : trajectories) {
//     control_msgs::FollowJointTrajectoryGoal goal;
//     buildControllerGoal(traj.first, traj.second, goal);
//     goals.push_back(goal);
//   }
//
//   std::vector<std::shared_ptr<FJT_actionClient>> clients;
//   for (const auto& traj : trajectories) {
//     std::shared_ptr<FJT_actionClient> client = control_clients_[traj.first];
//     clients.push_back(client);
//   }
//
//   for (size_t i = 0; i < clients.size(); ++i) {
//     clients[i]->sendGoal(goals[i]);
//   }
//
//   for (auto& client : clients) {
//     if (!client->waitForResult(ros::Duration(duration))) {
//       ROS_ERROR("RoPort: Trajectory execution timeout after %f seconds", duration);
//       return false;
//     }
//   }
//   return true;
// }
//
// void CartesIOServer::buildControllerGoal(int group_id,
//                                          const trajectory_msgs::JointTrajectory& trajectory,
//                                          control_msgs::FollowJointTrajectoryGoal& goal) {
//   goal.goal_time_tolerance = ros::Duration(1);
//   goal.goal_tolerance = buildTolerance(group_id, 0.01, 0, 0);
//   goal.path_tolerance = buildTolerance(group_id, 0.02, 0, 0);
//   goal.trajectory = trajectory;
// }
//
// auto CartesIOServer::buildTolerance(int group_id, double position, double velocity, double acceleration)
//     -> std::vector<control_msgs::JointTolerance> {
//   control_msgs::JointTolerance joint_tolerance;
//   joint_tolerance.position = position;
//   joint_tolerance.velocity = velocity;
//   joint_tolerance.acceleration = acceleration;
//   int joint_num = interfaces_[group_id]->getActiveJoints().size();
//   return std::vector<control_msgs::JointTolerance>(joint_num, joint_tolerance);
// }
//
// void CartesIOServer::geometryPoseToEigenMatrix(const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d& mat) {
//   mat = Eigen::Matrix4d::Identity();
//
//   Eigen::Quaterniond orientation;
//   orientation.coeffs() << pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
//       pose.pose.orientation.w;
//   Eigen::Quaterniond orientation_n = orientation.normalized();
//   Eigen::Matrix3d rotation_matrix = orientation_n.toRotationMatrix();
//   mat.topLeftCorner(3, 3) = rotation_matrix;
//
//   Eigen::Vector3d t_eigen;
//   t_eigen << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
//   mat.topRightCorner(3, 1) = t_eigen;
// }
//
// void CartesIOServer::eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::PoseStamped& pose) {
//   Eigen::Matrix3d rotation_matrix = mat.topLeftCorner(3, 3);
//   Eigen::Vector3d t_eigen = mat.topRightCorner(3, 1);
//   Eigen::Quaterniond quat(rotation_matrix);
//
//   pose.pose.position.x = t_eigen[0];
//   pose.pose.position.y = t_eigen[1];
//   pose.pose.position.z = t_eigen[2];
//   pose.pose.orientation.x = quat.x();
//   pose.pose.orientation.y = quat.y();
//   pose.pose.orientation.z = quat.z();
//   pose.pose.orientation.w = quat.w();
// }
//
// auto CartesIOServer::getGroupIndex(const std::string& group_name) -> int {
//   auto name = std::find(group_names_.begin(), group_names_.end(), group_name);
//   if (name != group_names_.end()) {
//     return name - group_names_.begin();
//   }
//   return -1;
// }
}  // namespace roport
