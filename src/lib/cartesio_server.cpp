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
    group_names_.push_back(std::string(group_names[i]));
    ROS_INFO("CartesIOServer: Added group '%s'", std::string(group_names[i]).c_str());
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
    ROS_INFO_STREAM("Listening the transform " << reference_frames_[i] << " -> " << controlled_frames_[i]);
  }

  XmlRpc::XmlRpcValue homing_poses;
  if (getParam(nh_, pnh_, "homing_poses", homing_poses)) {
    ROS_ASSERT(homing_poses.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(homing_poses.size() == group_names_.size());
    for (int i = 0; i < homing_poses.size(); ++i) {
      ROS_ASSERT(homing_poses[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(homing_poses[i].size() == 7);
      geometry_msgs::Pose pose;
      pose.position.x = homing_poses[i][0];
      pose.position.y = homing_poses[i][1];
      pose.position.z = homing_poses[i][2];
      pose.orientation.x = homing_poses[i][3];
      pose.orientation.y = homing_poses[i][4];
      pose.orientation.z = homing_poses[i][5];
      pose.orientation.w = homing_poses[i][6];
      homing_poses_.push_back(pose);
    }
  }

  // Timeout for waiting CartesI/O action servers to launch
  XmlRpc::XmlRpcValue timeout;
  getParam(nh_, pnh_, "timeout", timeout);
  ROS_ASSERT(timeout.getType() == XmlRpc::XmlRpcValue::TypeInt);
  ROS_INFO("CartesIO Server launching timeout: %is", int(timeout));

  // Initialize controller action clients, one for each group
  for (const auto& group_name : group_names_) {
    std::string action_name = "cartesian/" + group_name + "/reach";
    auto client = std::make_shared<reachPoseActionClient>(action_name);
    if (!client->waitForServer(ros::Duration(int(timeout)))) {
      throw std::runtime_error("RoPort: Action server " + action_name + " unavailable");
    }
    control_clients_.push_back(client);
    ROS_INFO_STREAM("Control action client " << action_name << " initialized");
  }

  // Services for getting control group information
  get_group_names_srv_ = nh_.advertiseService("get_group_names", &CartesIOServer::getGroupNamesCb, this);
  get_group_pose_srv_ = nh_.advertiseService("get_group_pose", &CartesIOServer::getGroupPoseCb, this);

  // Servers for controlling multiple groups' pose
  execute_all_poses_srv_ = nh_.advertiseService("execute_all_poses", &CartesIOServer::executeAllPosesSrvCb, this);
  execute_all_locked_poses_srv_ =
      nh_.advertiseService("execute_all_locked_poses", &CartesIOServer::executeAllLockedPosesSrvCb, this);

  // Servers for controlling one group's pose
  execute_group_homing_srv_ = nh_.advertiseService("execute_group_homing", &CartesIOServer::executeHomingSrvCb, this);
  execute_group_pose_srv_ = nh_.advertiseService("execute_group_pose", &CartesIOServer::executeGroupPoseCb, this);

  // Servers for controlling multiple groups' trajectories
  execute_trajectories_srv_ = nh_.advertiseService("execute_multiple_cartesian_trajectories",
                                                   &CartesIOServer::executeMultipleCartesianTrajectoriesCb, this);
}

bool CartesIOServer::checkGroupValid(const std::string& required_group_name, int& index) {
  index = getIndex(group_names_, required_group_name);
  if (index < 0) {
    ROS_ERROR_STREAM("No group named '" << required_group_name << "' defined");
    ROS_WARN("Defined group names are:");
    logWarningList<std::vector<std::string>>(group_names_);
    return false;
  }
  return true;
}

auto CartesIOServer::getGroupNamesCb(roport::GetAllNames::Request& req, roport::GetAllNames::Response& resp) -> bool {
  resp.group_names.resize(group_names_.size());
  std::copy(group_names_.begin(), group_names_.end(), resp.group_names.begin());
  return true;
}

auto CartesIOServer::getGroupPoseCb(roport::GetGroupPose::Request& req, roport::GetGroupPose::Response& resp) -> bool {
  int index = -1;
  if (!checkGroupValid(req.group_name, index)) {
    resp.result_status = roport::GetGroupPose::Response::FAILED;
    return false;
  }

  if (!getCurrentPoseWithIndex(index, resp.pose, req.ref_frame, req.ee_frame)) {
    resp.result_status = roport::GetGroupPose::Response::FAILED;
    return false;
  }

  resp.ref_link = req.ref_frame.empty() ? reference_frames_[index] : req.ref_frame;
  resp.ee_link = req.ee_frame.empty() ? controlled_frames_[index] : req.ee_frame;
  return true;
}

auto CartesIOServer::executeGroupPoseCb(roport::ExecuteGroupPose::Request& req,
                                        roport::ExecuteGroupPose::Response& resp) -> bool {
  int index = -1;
  if (!checkGroupValid(req.group_name, index)) {
    resp.result_status = roport::ExecuteGroupPose::Response::FAILED;
    return false;
  }

  if (req.duration == 0) {
    resp.result_status = roport::ExecuteGroupPose::Response::SUCCEEDED;
    resp.result_msg = "No execution as duration is 0";
    return true;
  }

  std::map<int, cartesian_interface::ReachPoseActionGoal> action_goals;
  geometry_msgs::Pose ref_to_ctrl_pose;
  if (!calculateReferenceToControlFrameGoalPose(index, req.ref_frame, req.ee_frame, req.goal, ref_to_ctrl_pose)) {
    resp.result_status = roport::ExecuteGroupPose::Response::FAILED;
    resp.result_msg = "Cannot get ref_to_ctrl pose";
    return false;
  }

  float duration = req.duration > 0 ? req.duration : 5.0;

  // Build trajectory to reach the goal
  cartesian_interface::ReachPoseActionGoal action_goal;
  buildActionGoal(index, ref_to_ctrl_pose, duration, action_goal);
  action_goals.insert({index, action_goal});

  if (executeGoals(action_goals)) {
    resp.result_status = roport::ExecuteGroupPose::Response::SUCCEEDED;
  } else {
    resp.result_status = roport::ExecuteGroupPose::Response::FAILED;
  }
  return true;
}

auto CartesIOServer::executeHomingSrvCb(roport::ExecuteGroupHoming::Request& req,
                                        roport::ExecuteGroupHoming::Response& resp) -> bool {
  int index = -1;
  if (!checkGroupValid(req.group_name, index)) {
    resp.result_status = roport::ExecuteGroupHoming::Response::FAILED;
    resp.result_msg = "Group name not valid";
    return false;
  }

  std::map<int, cartesian_interface::ReachPoseActionGoal> action_goals;

  geometry_msgs::Pose goal_pose = homing_poses_[index];
  float duration = req.duration > 0 ? req.duration : 10.0;

  // Build trajectory to reach the goal
  cartesian_interface::ReachPoseActionGoal action_goal;
  buildActionGoal(index, goal_pose, duration, action_goal);
  action_goals.insert({index, action_goal});

  if (executeGoals(action_goals)) {
    resp.result_status = roport::ExecuteGroupPose::Response::SUCCEEDED;
  } else {
    resp.result_status = roport::ExecuteGroupPose::Response::FAILED;
  }
  return true;
}

auto CartesIOServer::executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req,
                                          roport::ExecuteAllPoses::Response& resp) -> bool {
  ROS_ASSERT(req.group_names.size() == req.goals.poses.size());
  std::map<int, cartesian_interface::ReachPoseActionGoal> action_goals;

  for (int i = 0; i < group_names_.size(); ++i) {
    for (int j = 0; j < req.group_names.size(); ++j) {
      if (req.group_names[j] == group_names_[i]) {
        geometry_msgs::Pose current_pose;
        if (!getCurrentPoseWithIndex(i, current_pose)) {
          return false;
        }

        geometry_msgs::Pose goal_pose;
        toGlobalPose(req.goal_type, current_pose, req.goals.poses[j], goal_pose);

        // Build trajectory to reach the goal
        cartesian_interface::ReachPoseActionGoal action_goal;
        buildActionGoal(i, goal_pose, 10.0, action_goal);
        if (req.stamps.size() == req.group_names.size() && req.stamps[j] > 0) {
          updateStamp(req.stamps[j], action_goal);
        }
        action_goals.insert({i, action_goal});
      }
    }
  }
  if (executeGoals(action_goals)) {
    resp.result_status = roport::ExecuteAllPoses::Response::SUCCEEDED;
  } else {
    resp.result_status = roport::ExecuteAllPoses::Response::FAILED;
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
      ROS_ERROR_STREAM("No group named '" << group_names_[j] << "' defined");
      return false;
    }

    geometry_msgs::Pose goal_pose;
    if (j == 0) {
      if (!getCurrentPoseWithIndex(i, current_reference_pose)) {
        return false;
      }
      reference_index = i;
      toGlobalPose(req.goal_type, current_reference_pose, req.goal, goal_pose);
      goal_reference_pose = goal_pose;
    } else {
      geometry_msgs::Pose current_pose;
      if (!getCurrentPoseWithIndex(i, current_pose)) {
        return false;
      }
      getGoalPoseWithReference(reference_index, current_reference_pose, goal_reference_pose, i, current_pose,
                               goal_pose);
    }
    // Build trajectory to reach the goal
    cartesian_interface::ReachPoseActionGoal action_goal;
    buildActionGoal(i, goal_pose, 10.0, action_goal);
    if (req.stamp > 0) {
      updateStamp(req.stamp, action_goal);
    }
    action_goals.insert({i, action_goal});
  }

  if (executeGoals(action_goals)) {
    resp.result_status = roport::ExecuteAllLockedPoses::Response::SUCCEEDED;
  } else {
    resp.result_status = roport::ExecuteAllLockedPoses::Response::FAILED;
  }
  return true;
}

auto CartesIOServer::executeMultipleCartesianTrajectoriesCb(ExecuteAllCartesianTrajectories::Request& req,
                                                            ExecuteAllCartesianTrajectories::Response& resp) -> bool {
  std::map<int, cartesian_interface::ReachPoseActionGoal> action_goals;
  for (size_t i = 0; i < req.group_names.size(); ++i) {
    int index = -1;
    auto controlled_group_name = req.group_names[i];
    if (!checkGroupValid(controlled_group_name, index)) {
      resp.result_msg = "Group name not valid";
      resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
      return false;
    }

    auto trajectory = req.trajectories[i];
    if (trajectory.points.empty()) {
      ROS_ERROR("Trajectory for group %s is empty", controlled_group_name.c_str());
      resp.result_msg = "Empty trajectory";
      resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
      return false;
    }

    cartesian_interface::ReachPoseActionGoal action_goal;
    for (size_t j = 0; j < trajectory.points.size(); j++) {
      auto point = trajectory.points[j];
      geometry_msgs::Pose ref_to_ctrl_pose;
      calculateReferenceToControlFrameGoalPose(index, trajectory.ref_frame, trajectory.ee_frame, point.pose,
                                               ref_to_ctrl_pose);

      buildActionGoal(index, ref_to_ctrl_pose, point.duration, action_goal);
    }
    action_goals.insert({index, action_goal});
  }

  if (executeGoals(action_goals)) {
    resp.result_status = roport::ExecuteGroupPose::Response::SUCCEEDED;
  } else {
    resp.result_status = roport::ExecuteGroupPose::Response::FAILED;
  }

  return true;
}

bool CartesIOServer::getCurrentPoseWithIndex(const int& index,
                                             geometry_msgs::Pose& pose,
                                             const std::string& reference_frame,
                                             const std::string& control_frame) {
  auto ref_frame = reference_frame.empty() ? reference_frames_[index] : reference_frame;
  auto ctrl_frame = control_frame.empty() ? controlled_frames_[index] : control_frame;

  geometry_msgs::TransformStamped ref_t_ctrl_stamped;
  if (!roport::getTransformWithTFBuffer(tf_buffer_, ref_frame, ctrl_frame, ref_t_ctrl_stamped)) {
    return false;
  }
  roport::geometryTransformToPose(ref_t_ctrl_stamped.transform, pose);
  return true;
}

bool CartesIOServer::calculateReferenceToControlFrameGoalPose(const int& index,
                                                              const std::string& user_ref_frame,
                                                              const std::string& user_ctrl_frame,
                                                              const geometry_msgs::Pose& raw_pose,
                                                              geometry_msgs::Pose& output_pose) {
  auto ref_frame = reference_frames_[index];
  auto ctrl_frame = controlled_frames_[index];

  geometry_msgs::TransformStamped ref_T_user_ref_stamped;
  if (user_ref_frame.empty()) {
    ROS_WARN_STREAM("User reference frame was not given, using " << ref_frame);
    ref_T_user_ref_stamped.transform = roport::identityTransform();
  } else {
    if (!roport::getTransformWithTFBuffer(tf_buffer_, ref_frame, user_ref_frame, ref_T_user_ref_stamped)) {
      return false;
    }
  }

  geometry_msgs::TransformStamped user_ctrl_T_ctrl_stamped;
  if (user_ctrl_frame.empty()) {
    ROS_WARN_STREAM("User control frame was not given, using " << ctrl_frame);
    user_ctrl_T_ctrl_stamped.transform = roport::identityTransform();
  } else {
    if (!roport::getTransformWithTFBuffer(tf_buffer_, user_ctrl_frame, ctrl_frame, user_ctrl_T_ctrl_stamped)) {
      return false;
    }
  }

  roport::getReferenceToControlledFramePose(ref_T_user_ref_stamped.transform, user_ctrl_T_ctrl_stamped.transform,
                                            raw_pose, output_pose);
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

void CartesIOServer::buildActionGoal(const int& index,
                                     const geometry_msgs::Pose& goal_pose,
                                     const float& duration,
                                     cartesian_interface::ReachPoseActionGoal& action_goal) {
  ROS_INFO("Building action goal with the following goal poses:");
  logROSPose(goal_pose);

  action_goal.header.frame_id = reference_frames_[index];
  action_goal.goal.frames.push_back(goal_pose);

  // Ref: https://advrhumanoids.github.io/CartesianInterface/tasks/cartesianros.html#reach
  // The time represents waypoints respective times w.r.t. trajectory start
  float d = duration > 0 ? duration : 10.0;
  float absolute_time = 0.0;
  for (const auto& t : action_goal.goal.time) {
    absolute_time += t;
  }
  action_goal.goal.time.push_back(absolute_time + d);

  // The incremental flag, if set to true, allows to specify waypoints w.r.t. the starting pose of the robot.
  action_goal.goal.incremental = false;
}

void CartesIOServer::updateStamp(const double& stamp, cartesian_interface::ReachPoseActionGoal& action_goal) {
  for (float& i : action_goal.goal.time) {
    i = static_cast<float>(stamp);
  }
}

auto CartesIOServer::executeGoals(const std::map<int, cartesian_interface::ReachPoseActionGoal>& goal_handlers)
    -> bool {
  std::map<int, float> duration_handlers;

  for (const auto& goal_handler : goal_handlers) {
    float total_duration = 0;
    for (const float& i : goal_handler.second.goal.time) {
      total_duration += i;
    }
    duration_handlers.insert({goal_handler.first, total_duration});
  }

  for (const auto& goal : goal_handlers) {
    // Send all goals for all control groups
    for (size_t i = 0; i < goal.second.goal.frames.size(); ++i) {
      control_clients_[goal.first]->sendGoal(goal.second.goal);
    }
  }

  for (const auto& duration_handler : duration_handlers) {
    auto extended_duration = duration_handler.second + 0.5;
    if (!control_clients_[duration_handler.first]->waitForResult(ros::Duration(extended_duration))) {
      ROS_ERROR("Goal(s) of group %s execution timeout (expected: %.2f seconds)",
                group_names_[duration_handler.first].c_str(), extended_duration);
      control_clients_[duration_handler.first]->cancelAllGoals();
      return false;
    }
  }

  return true;
}
}  // namespace roport
