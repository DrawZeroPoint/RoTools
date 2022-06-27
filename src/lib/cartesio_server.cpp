//
// Created by dzp on 2020/10/14.
//

#include "roport/cartesio_server.h"

#include "roport/common.h"

namespace roport {

CartesIOServer::CartesIOServer(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh), tf_listener_(tf_buffer_) {
  // Get all available planning group names
  XmlRpc::XmlRpcValue group_names;
  getParam(nh_, pnh_, "group_names", group_names);
  ROS_ASSERT(group_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (group_names.size() == 0) {
    throw std::runtime_error("No group name");
  }
  for (int i = 0; i < group_names.size(); i++) {
    ROS_ASSERT(group_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    group_names_.push_back(group_names[i]);
  }

  XmlRpc::XmlRpcValue controlled_frames;
  getParam(nh_, pnh_, "controlled_frames", controlled_frames);
  ROS_ASSERT(controlled_frames.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (controlled_frames.size() != group_names_.size()) {
    throw std::runtime_error("Controlled frames size does not match group names");
  }
  for (int i = 0; i < controlled_frames.size(); i++) {
    ROS_ASSERT(controlled_frames[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    controlled_frames_.push_back(controlled_frames[i]);
  }

  XmlRpc::XmlRpcValue reference_frames;
  getParam(nh_, pnh_, "reference_frames", reference_frames);
  ROS_ASSERT(reference_frames.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (reference_frames.size() != controlled_frames_.size()) {
    throw std::runtime_error("Reference frames size does not match controlled frames");
  }
  for (int i = 0; i < reference_frames.size(); i++) {
    ROS_ASSERT(reference_frames[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    reference_frames_.push_back(reference_frames[i]);
    ROS_INFO_STREAM("listening the transform " << reference_frames_[i] << " -> " << controlled_frames_[i]);
  }

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
  execute_all_poses_srv_ = nh_.advertiseService("execute_all_poses", &CartesIOServer::executeAllPosesSrvCb, this);
  //  execute_mirrored_pose_srv_ =
  //      nh_.advertiseService("execute_mirrored_pose", &CartesIOServer::executeMirroredPoseSrvCb, this);
}

auto CartesIOServer::executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req,
                                          roport::ExecuteAllPoses::Response& resp) -> bool {
  ROS_ASSERT(req.group_names.size() == req.goals.poses.size());
  std::vector<cartesian_interface::ReachPoseActionGoal> action_goals;

  for (int i = 0; i < group_names_.size(); ++i) {
    for (int j = 0; j < req.group_names.size(); ++j) {
      if (req.group_names[j] == group_names_[i]) {
        geometry_msgs::TransformStamped current_transform;
        if (!getTransform(i, current_transform)) {
          return false;
        }

        geometry_msgs::Pose current_pose;
        current_pose.position.x = current_transform.transform.translation.x;
        current_pose.position.y = current_transform.transform.translation.y;
        current_pose.position.z = current_transform.transform.translation.z;
        current_pose.orientation.x = current_transform.transform.rotation.x;
        current_pose.orientation.y = current_transform.transform.rotation.y;
        current_pose.orientation.z = current_transform.transform.rotation.z;
        current_pose.orientation.w = current_transform.transform.rotation.w;

        // Initialize the goal pose
        geometry_msgs::Pose goal_pose;

        // Transfer the given pose to the reference frame
        if (req.goal_type == req.BASE_ABS) {
          // The given pose is already in the reference frame
          goal_pose = req.goals.poses[j];
        } else if (req.goal_type == req.BASE_REL) {
          // The given pose is relative to the local aligned frame having the same orientation as the reference frame
          localAlignedPoseToGlobalPose(req.goals.poses[j], current_pose, goal_pose);
        } else {
          // The given pose is relative to the local frame
          localPoseToGlobalPose(req.goals.poses[j], current_pose, goal_pose);
        }

        // Build trajectory to reach the goal
        cartesian_interface::ReachPoseActionGoal action_goal;
        buildActionGoal(i, goal_pose, action_goal);
        if (req.stamps.size() == req.group_names.size() && req.stamps[j] > 0) {
          updateStamp(req.stamps[j], action_goal);
          action_goals.push_back(action_goal);
        } else {
          action_goals.push_back(action_goal);
        }
      }
    }
  }
  if (executeGoals(action_goals)) {
    resp.result_status = resp.SUCCEEDED;
  } else {
    resp.result_status = resp.FAILED;
  }
  return true;
}

bool CartesIOServer::getTransform(const int& index, geometry_msgs::TransformStamped& transform) {
  try {
    transform = tf_buffer_.lookupTransform(controlled_frames_[index], reference_frames_[index], ros::Time(0));
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

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

void CartesIOServer::buildActionGoal(const int& index,
                                     const geometry_msgs::Pose& goal_pose,
                                     cartesian_interface::ReachPoseActionGoal& action_goal) {
  action_goal.header.frame_id = reference_frames_[index];
  action_goal.goal.frames.push_back(goal_pose);
  action_goal.goal.time.push_back(1.);
  action_goal.goal.incremental = true;
}

void CartesIOServer::updateStamp(const double& stamp, cartesian_interface::ReachPoseActionGoal& action_goal) {
  for (float& i : action_goal.goal.time) {
    i = static_cast<float>(stamp);
  }
}

auto CartesIOServer::executeGoals(const std::vector<cartesian_interface::ReachPoseActionGoal>& goals, double duration)
    -> bool {
  for (size_t i = 0; i < control_clients_.size(); ++i) {
    control_clients_[i]->sendGoal(goals[i].goal);
  }

  for (size_t i = 0; i < control_clients_.size(); ++i) {
    if (!control_clients_[i]->waitForResult(ros::Duration(duration))) {
      ROS_ERROR("Goal %zu of group %s execution timeout after %f seconds", i, group_names_[i].c_str(), duration);
      return false;
    }
  }
  return true;
}

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
}  // namespace roport