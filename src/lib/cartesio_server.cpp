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
    ROS_INFO_STREAM("Control action client " << action_name << " initialized");
  }

  // Initialize control servers
  execute_all_poses_srv_ = nh_.advertiseService("execute_all_poses", &CartesIOServer::executeAllPosesSrvCb, this);
  execute_all_locked_poses_srv_ =
      nh_.advertiseService("execute_all_locked_poses", &CartesIOServer::executeAllLockedPosesSrvCb, this);
  //  execute_mirrored_pose_srv_ =
  //      nh_.advertiseService("execute_mirrored_pose", &CartesIOServer::executeMirroredPoseSrvCb, this);
}

auto CartesIOServer::executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req,
                                          roport::ExecuteAllPoses::Response& resp) -> bool {
  ROS_ASSERT(req.group_names.size() == req.goals.poses.size());
  std::map<int, cartesian_interface::ReachPoseActionGoal> action_goals;

  for (int i = 0; i < group_names_.size(); ++i) {
    for (int j = 0; j < req.group_names.size(); ++j) {
      if (req.group_names[j] == group_names_[i]) {
        geometry_msgs::Pose current_pose;
        if (!getPose(i, current_pose)) {
          return false;
        }

        // Initialize the goal pose
        geometry_msgs::Pose goal_pose;

        // Transfer the given pose to the reference frame
        if (req.goal_type == req.GLOBAL) {
          // The given pose is already in the reference frame
          goal_pose = req.goals.poses[j];
        } else if (req.goal_type == req.LOCAL_ALIGNED) {
          // The given pose is relative to the local aligned frame having the same orientation as the reference frame
          localAlignedPoseToGlobalPose(req.goals.poses[j], current_pose, goal_pose, true);
        } else {
          // The given pose is relative to the local frame
          localPoseToGlobalPose(req.goals.poses[j], current_pose, goal_pose);
        }

        // Build trajectory to reach the goal
        cartesian_interface::ReachPoseActionGoal action_goal;
        buildActionGoal(i, goal_pose, action_goal);
        if (req.stamps.size() == req.group_names.size() && req.stamps[j] > 0) {
          updateStamp(req.stamps[j], action_goal);
        }
        action_goals.insert({i, action_goal});
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

auto CartesIOServer::executeAllLockedPosesSrvCb(roport::ExecuteAllLockedPoses::Request& req,
                                                roport::ExecuteAllLockedPoses::Response& resp) -> bool {
  std::map<int, cartesian_interface::ReachPoseActionGoal> action_goals;

  int reference_index = -1;
  geometry_msgs::Pose current_reference_pose;
  geometry_msgs::Pose goal_reference_pose;
  for (int j = 0; j < req.group_names.size(); ++j) {
    int i = getIndex(group_names_, req.group_names[j]);
    if (i < 0) {
      ROS_ERROR_STREAM("No " << group_names_[j] << " group defined");
      return false;
    }

    geometry_msgs::Pose goal_pose;
    if (j == 0) {
      if (!getPose(i, current_reference_pose)) {
        return false;
      }
      reference_index = i;

      // Transfer the given pose to the reference frame
      if (req.goal_type == req.GLOBAL) {
        // The given pose is already in the reference frame
        goal_pose = req.goal;
      } else if (req.goal_type == req.LOCAL_ALIGNED) {
        // The given pose is relative to the local aligned frame having the same orientation as the reference frame
        localAlignedPoseToGlobalPose(req.goal, current_reference_pose, goal_pose, true);
      } else {
        // The given pose is relative to the local frame
        localPoseToGlobalPose(req.goal, current_reference_pose, goal_pose);
      }
      goal_reference_pose = goal_pose;
    } else {
      geometry_msgs::Pose current_pose;
      if (!getPose(i, current_pose)) {
        return false;
      }
      getGoalPoseWithReference(reference_index, current_reference_pose, goal_reference_pose, i, current_pose,
                               goal_pose);
    }
    // Build trajectory to reach the goal
    cartesian_interface::ReachPoseActionGoal action_goal;
    buildActionGoal(i, goal_pose, action_goal);
    if (req.stamp > 0) {
      updateStamp(req.stamp, action_goal);
    }
    action_goals.insert({i, action_goal});
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
    // The transform derived by lookupTransform is T_rc, i.e., from the reference to the controlled frame.
    transform = tf_buffer_.lookupTransform(reference_frames_[index], controlled_frames_[index], ros::Time(0));
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

bool CartesIOServer::getPose(const int& index, geometry_msgs::Pose& pose) {
  geometry_msgs::TransformStamped transform;
  if (!getTransform(index, transform)) {
    return false;
  }

  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation.x = transform.transform.rotation.x;
  pose.orientation.y = transform.transform.rotation.y;
  pose.orientation.z = transform.transform.rotation.z;
  pose.orientation.w = transform.transform.rotation.w;
  return true;
}

void CartesIOServer::getGoalPoseWithReference(const int& ref_idx,
                                              const geometry_msgs::Pose& curr_ref_pose,
                                              const geometry_msgs::Pose& goal_ref_pose,
                                              const int& idx,
                                              const geometry_msgs::Pose& curr_pose,
                                              geometry_msgs::Pose& goal_pose) {
  // TODO release this constraint
  ROS_ASSERT(reference_frames_[ref_idx] == reference_frames_[idx]);
  Eigen::Matrix4d trans_b_rc, trans_b_rg, trans_b_c;
  geometryPoseToEigenMatrix(curr_ref_pose, trans_b_rc);
  geometryPoseToEigenMatrix(goal_ref_pose, trans_b_rg);
  geometryPoseToEigenMatrix(curr_pose, trans_b_c);

  // T_B_G = T_B_RG * T_RC_B * T_B_C
  auto trans_b_g = trans_b_rg * trans_b_rc.inverse() * trans_b_c;
  eigenMatrixToGeometryPose(trans_b_g, goal_pose);
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
  action_goal.goal.time.push_back(10.);
  action_goal.goal.incremental = false;
}

void CartesIOServer::updateStamp(const double& stamp, cartesian_interface::ReachPoseActionGoal& action_goal) {
  for (float& i : action_goal.goal.time) {
    i = static_cast<float>(stamp);
  }
}

auto CartesIOServer::executeGoals(const std::map<int, cartesian_interface::ReachPoseActionGoal>& goals, double duration)
    -> bool {
  for (const auto& goal : goals) {
    control_clients_[goal.first]->sendGoal(goal.second.goal);
  }

  for (const auto& goal : goals) {
    if (!control_clients_[goal.first]->waitForResult(ros::Duration(duration))) {
      ROS_ERROR("Goal of group %s execution timeout after %f seconds", group_names_[goal.first].c_str(), duration);
      return false;
    }
  }
  return true;
}
}  // namespace roport
