#include "roport/trajectory_planner.h"

namespace roport {

CartesianTrajectoryPlanner::CartesianTrajectoryPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), visualize_(false), is_execute_(true) {
  // Get all available planning group names
  XmlRpc::XmlRpcValue group_names;
  roport::getParam(nh_, pnh_, "group_names", group_names);
  ROS_ASSERT(group_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (group_names.size() == 0) {
    throw std::runtime_error("Param group_names not defined");
  }

  // Create clients for getting the current group pose and executing pose command
  XmlRpc::XmlRpcValue get_group_pose_service_id;
  roport::getParam(nh_, pnh_, "get_group_pose_service_id", get_group_pose_service_id);
  ROS_ASSERT(get_group_pose_service_id.size() == group_names.size());

  XmlRpc::XmlRpcValue execute_group_cartesian_trajectory_service_id;
  roport::getParam(nh_, pnh_, "execute_group_cartesian_trajectory_service_id",
                   execute_group_cartesian_trajectory_service_id);
  ROS_ASSERT(execute_group_cartesian_trajectory_service_id.size() == group_names.size());

  for (int i = 0; i < group_names.size(); i++) {
    ROS_ASSERT(group_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string name = std::string(group_names[i]);

    std::string service_id = std::string(get_group_pose_service_id[i]);
    auto get_pose_client = nh_.serviceClient<roport::GetGroupPose>(service_id);
    if (!ros::service::waitForService(service_id, wait_for_service_timeout_)) {
      throw std::runtime_error("Failed to connect to get_group_pose service");
    }
    ROS_INFO("Created client for group '%s' to service %s", name.c_str(), service_id.c_str());

    service_id = std::string(execute_group_cartesian_trajectory_service_id[i]);
    auto execute_trajectory_client = nh_.serviceClient<roport::ExecuteGroupCartesianTrajectory>(service_id);
    if (!ros::service::waitForService(service_id, wait_for_service_timeout_)) {
      ROS_WARN_STREAM("Failed to connect to service: " << service_id);
      is_execute_ = false;
    } else {
      ROS_INFO("Created client for group '%s' to service %s", name.c_str(), service_id.c_str());
    }

    group_names_.push_back(name);
    get_current_pose_clients_.push_back(get_pose_client);
    execute_group_cartesian_trajectory_clients_.push_back(execute_trajectory_client);
  }

  // Get visualize flag
  XmlRpc::XmlRpcValue display_trajectory;
  roport::getParam(nh_, pnh_, "display_trajectory", display_trajectory);
  visualize_ = bool(display_trajectory);

  // Get joint names for each planning group
  XmlRpc::XmlRpcValue joint_names;
  roport::getParam(nh_, pnh_, "joint_names", joint_names);
  if (joint_names.size() != group_names.size()) {
    throw std::runtime_error("Param joint_names is not properly defined");
  }
  for (int i = 0; i < group_names_.size(); ++i) {
    ROS_ASSERT(joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<std::string> joint_name_list;
    for (int j = 0; j < joint_names[i].size(); ++j) {
      joint_name_list.push_back(joint_names[i][j]);
    }
    joint_names_.push_back(joint_name_list);
  }

  execute_all_cartesian_trajectory_srv_ = nh_.advertiseService(
      "execute_all_cartesian_trajectories", &CartesianTrajectoryPlanner::executeAllCartesianTrajectoriesCb, this);

  if (visualize_) {
    joint_state_subscriber_ =
        nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &CartesianTrajectoryPlanner::jointStatesCb, this);

    cartesian_trajectory_publisher_ =
        nh_.advertise<geometry_msgs::PoseArray>("trajectory_planner/cartesian_trajectory", 1);
    ROS_INFO("Planned cartesian trajectory can be visualized in RViz with: trajectory_planner/cartesian_trajectory");

    display_trajectory_publisher_ =
        nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory_planner/joint_trajectory", 1);
    ROS_INFO("Planned joint trajectory can be visualized in RViz with: trajectory_planner/joint_trajectory");
  }
}

void CartesianTrajectoryPlanner::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
  current_joint_state_ = *msg;

  joint_positions_.clear();
  for (int i = 0; i < joint_names_.size(); ++i) {
    std::vector<double> position_list;
    for (int j = 0; j < joint_names_[i].size(); ++j) {
      auto res = findInVector<std::string>(msg->name, joint_names_[i][j]);
      if (res.first) {
        position_list.push_back(msg->position[res.second]);
      } else {
        throw std::runtime_error("Joint name not found in joint_states");
      }
    }
    joint_positions_.push_back(position_list);
  }
}

auto CartesianTrajectoryPlanner::executeAllCartesianTrajectoriesCb(
    roport::ExecuteAllCartesianTrajectories::Request& req,
    roport::ExecuteAllCartesianTrajectories::Response& resp) -> bool {
  if (req.group_names.empty()) {
    ROS_WARN("The 'group_names' in the execute all cartesian trajectory request is empty.");
    resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
    resp.result_msg = "Empty group_names";
    return true;
  }

  std::map<int, roport::CartesianTrajectory> trajectory_handler;
  for (size_t i = 0; i < req.group_names.size(); ++i) {
    auto result = roport::findInVector(group_names_, req.group_names[i]);
    if (!result.first) {
      ROS_WARN("Group name %s is not in known names, its trajectory will not be executed", req.group_names[i].c_str());
      resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
      resp.result_msg = "Invalid group name";
      return true;
    }

    auto index = result.second;
    auto reference_frame = req.trajectories[i].ref_frame;
    auto ee_frame = req.trajectories[i].ee_frame;

    roport::GetGroupPose get_current_pose;
    get_current_pose.request.group_name = group_names_[index];
    get_current_pose.request.ref_frame = reference_frame;
    get_current_pose.request.ee_frame = ee_frame;
    // Use the current pose as the initial commanded pose of the generated cartesian trajectory.
    get_current_pose_clients_[index].call(get_current_pose);

    if (get_current_pose.response.result_status == roport::GetGroupPoseResponse::SUCCEEDED) {
      roport::CartesianTrajectory goal_trajectory;
      CartesianTrajectoryPlanner::makeCartesianTrajectoryWithDrake(get_current_pose.response.pose, req.trajectories[i],
                                                                   goal_trajectory);
      if (visualize_) {
        displayCartesianTrajectoryInRViz(goal_trajectory);
      }
      trajectory_handler.insert({index, goal_trajectory});
    } else {
      ROS_WARN("Fail to get the current pose for group %s", group_names_[index].c_str());
      resp.result_status = roport::ExecuteAllCartesianTrajectoriesResponse::FAILED;
      resp.result_msg = "Get current pose failed";
      return false;
    }
  }

  /** This naive implementation only allows executing one trajectory for one group for testing
  for (const auto& pair : trajectory_handler) {
    roport::ExecuteGroupPose execute_group_pose;
    execute_group_pose.request.ref_frame = pair.second.ref_frame;
    execute_group_pose.request.ee_frame = pair.second.ee_frame;
    execute_group_pose.request.group_name = group_names_[pair.first];
    for (int i = 0; i < pair.second.points.size(); i += 100) {
      auto p = pair.second.points[i];
      execute_group_pose.request.goal = p.pose;
      execute_group_pose.request.duration = p.duration * 100;
      execute_group_pose_clients_[pair.first].call(execute_group_pose);
      if (execute_group_pose.response.result_status != roport::ExecuteGroupPose::Response::SUCCEEDED) {
        ROS_ERROR("Call execute_group_pose service failed");
        break;
      }
    }
  }
  **/
  if (is_execute_) {
    // TODO use execute_group_cartesian_trajectory_clients_ to send dense trajectories to each group
  }
  resp.result_status = roport::ExecuteAllCartesianTrajectoriesResponse::SUCCEEDED;
  return true;
}

void CartesianTrajectoryPlanner::geometryPoseToDrakeRigidTransform(const geometry_msgs::Pose& p,
                                                                   drake::math::RigidTransformd& t) {
  Eigen::Matrix4d m;
  geometryPoseToEigenMatrix(p, m);
  t = drake::math::RigidTransformd(m);
}

void CartesianTrajectoryPlanner::drakeRigidTransformToGeometryPose(const drake::math::RigidTransformd& t,
                                                                   geometry_msgs::Pose& p) {
  Eigen::Matrix4d m = t.GetAsMatrix4();
  eigenMatrixToGeometryPose(m, p);
}

void CartesianTrajectoryPlanner::drakeRigidTransformToCartesianTrajectoryPoint(const drake::math::RigidTransformd& pose,
                                                                               const Eigen::Matrix<double, 6, 1>& vel,
                                                                               const Eigen::Matrix<double, 6, 1>& acc,
                                                                               const double& duration,
                                                                               roport::CartesianTrajectoryPoint& p) {
  drakeRigidTransformToGeometryPose(pose, p.pose);
  eigenMatrixToGeometryTwist(vel, p.twist);
  eigenMatrixToGeometryAccel(acc, p.acceleration);
  p.duration = duration;
}

void CartesianTrajectoryPlanner::drakeTrajectoryToCartesianTrajectory(
    const drake::trajectories::PiecewisePose<double>& drake_trajectory,
    roport::CartesianTrajectory& trajectory) const {
  ROS_WARN_STREAM(drake_trajectory.start_time() << "  " << drake_trajectory.end_time());
  int trajectory_length = static_cast<int>(drake_trajectory.end_time() / default_time_step_);
  for (int i = 0; i <= trajectory_length; ++i) {
    auto time_stamp = default_time_step_ * i;
    auto pose = drake_trajectory.GetPose(time_stamp);
    auto vel = drake_trajectory.GetVelocity(time_stamp);
    auto acc = drake_trajectory.GetAcceleration(time_stamp);
    roport::CartesianTrajectoryPoint point;
    if (i == 0) {
      drakeRigidTransformToCartesianTrajectoryPoint(pose, vel, acc, 0.0, point);
      logROSPose(point.pose);
    } else {
      drakeRigidTransformToCartesianTrajectoryPoint(pose, vel, acc, default_time_step_, point);
      if (i == trajectory_length) {
        logROSPose(point.pose);
      }
    }
    trajectory.points.push_back(point);
  }
}

bool CartesianTrajectoryPlanner::makeCartesianTrajectoryWithDrake(geometry_msgs::Pose initial_pose,
                                                                  const roport::CartesianTrajectory& sparse_trajectory,
                                                                  roport::CartesianTrajectory& goal_trajectory) {
  goal_trajectory.header = sparse_trajectory.header;
  goal_trajectory.ref_frame = sparse_trajectory.ref_frame;
  goal_trajectory.ee_frame = sparse_trajectory.ee_frame;

  std::vector<double> times;
  std::vector<drake::math::RigidTransformd> poses;

  // The times for Drake's PiecewisePose record time_from_start, successive stamps must satisfy:
  // breaks_[i] - breaks_[i - 1] >= kEpsilonTime (i.e., std::numeric_limits<T>::epsilon = 2.22045e-16)
  double time_from_start = 0.0;
  times.push_back(time_from_start);  // The time_stamp for the initial_pose
  drake::math::RigidTransformd initial_t;
  geometryPoseToDrakeRigidTransform(initial_pose, initial_t);
  poses.push_back(initial_t);

  for (const auto& p : sparse_trajectory.points) {
    time_from_start += p.duration;
    times.push_back(time_from_start);
    drake::math::RigidTransformd intermediate_t;
    geometryPoseToDrakeRigidTransform(p.pose, intermediate_t);
    poses.push_back(intermediate_t);
  }

  auto drake_trajectory = drake::trajectories::PiecewisePose<double>::MakeLinear(times, poses);
  drakeTrajectoryToCartesianTrajectory(drake_trajectory, goal_trajectory);
  return true;
}

void CartesianTrajectoryPlanner::displayCartesianTrajectoryInRViz(
    const roport::CartesianTrajectory& cartesian_trajectory,
    const int& step) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = cartesian_trajectory.ref_frame;
  for (int i = 0; i < cartesian_trajectory.points.size(); i += step) {
    pose_array.poses.push_back(cartesian_trajectory.points[i].pose);
  }
  cartesian_trajectory_publisher_.publish(pose_array);
}

void CartesianTrajectoryPlanner::displayJointTrajectoryInRViz(const int& index,
                                                              const roport::CartesianTrajectory& cartesian_trajectory) {
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.model_id = "";

  moveit_msgs::RobotState trajectory_start;
  trajectory_start.joint_state = current_joint_state_;
  display_trajectory.trajectory_start = trajectory_start;

  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names = joint_names_[index];
  //
  display_trajectory.trajectory.push_back(robot_trajectory);

  display_trajectory_publisher_.publish(display_trajectory);
}
}  // namespace roport
