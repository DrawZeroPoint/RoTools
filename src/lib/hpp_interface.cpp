#include <memory>

#include "roport/common.h"
#include "roport/hpp_interface.h"

namespace roport {
PathPlanningInterface::PathPlanningInterface(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh)
    : nh_(node_handle),
      pnh_(pnh),
      is_initial_state_set_(false),
      is_initial_location_set_(false),
      time_step_(kDefaultStep),
      position_tolerance_(kPositionTolerance),
      orientation_tolerance_(kOrientationTolerance) {
  path_planning_solver_ = ProblemSolver::create();

  if (!createRobot()) {
    return;
  }
  path_planning_solver_->robot(robot_);

  // Set locomotion area
  if (!setBound()) {
    return;
  }
  // Load a URDF describing the environment and obstacles in it
  if (!createObstacle()) {
    ROS_WARN("No obstacle loaded");
  }

  bool loaded;
  try {
    std::string filename = plugin::findPluginLibrary("spline-gradient-based.so");
    loaded = plugin::loadPlugin(filename, path_planning_solver_);
  } catch (const std::invalid_argument&) {
    loaded = false;
  }
  if (loaded) {
    ROS_INFO("Using path optimizer: SplineGradientBased_bezier1");
    path_planning_solver_->addPathOptimizer("SplineGradientBased_bezier1");
  } else {
    ROS_WARN("Could not load spline-gradient-based.so, using path optimizer: RandomShortcut");
    path_planning_solver_->addPathOptimizer("RandomShortcut");
  }

  XmlRpc::XmlRpcValue state_topic_id;
  if (!getParam(nh_, pnh_, "state_topic_id", state_topic_id)) {
    return;
  }
  current_state_subscriber_ = nh_.subscribe<sensor_msgs::JointState>(
      state_topic_id, 1, [this](auto&& ph1) { return currentJointConfigCb(std::forward<decltype(ph1)>(ph1)); });

  XmlRpc::XmlRpcValue location_topic_id;
  if (!getParam(nh_, pnh_, "location_topic_id", location_topic_id)) {
    return;
  }
  current_location_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      location_topic_id, 1, [this](auto&& ph1) { return currentLocationCb(std::forward<decltype(ph1)>(ph1)); });

  XmlRpc::XmlRpcValue execute_path_planning_srv_id;
  if (!getParam(nh_, pnh_, "execute_path_planning_srv_id", execute_path_planning_srv_id)) {
    return;
  }
  execute_path_planning_srv_ =
      nh_.advertiseService(execute_path_planning_srv_id, &PathPlanningInterface::executePathPlanningSrvCb, this);

  XmlRpc::XmlRpcValue joint_command_topic_id;
  if (!getParam(nh_, pnh_, "joint_command_topic_id", joint_command_topic_id)) {
    return;
  }
  joint_command_publisher_ = nh_.advertise<sensor_msgs::JointState>(joint_command_topic_id, 1);

  XmlRpc::XmlRpcValue base_vel_cmd_topic_id;
  if (!getParam(nh_, pnh_, "base_vel_cmd_topic_id", base_vel_cmd_topic_id)) {
    return;
  }
  base_vel_cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>(base_vel_cmd_topic_id, 1);
  ROS_INFO("Robot HPP interface ready");
}

auto PathPlanningInterface::createRobot() -> bool {
  XmlRpc::XmlRpcValue robot_name;
  if (!getParam(nh_, pnh_, "robot_name", robot_name)) {
    return false;
  }

  XmlRpc::XmlRpcValue root_joint;
  if (!getParam(nh_, pnh_, "root_joint", root_joint)) {
    return false;
  }
  root_joint_type_ = std::string(root_joint);

  XmlRpc::XmlRpcValue robot_pkg_name;
  if (!getParam(nh_, pnh_, "robot_pkg_name", robot_pkg_name)) {
    return false;
  }

  XmlRpc::XmlRpcValue model_name;
  if (!getParam(nh_, pnh_, "model_name", model_name)) {
    return false;
  }

  robot_ = hpp_pin::Device::create(std::string(robot_name));
  hpp_pin::urdf::loadRobotModel(robot_, root_joint_type_, robot_pkg_name, model_name, "", "");
  robot_->controlComputation(static_cast<Computation_t>(JOINT_POSITION | JACOBIAN));
  q_current_ = robot_->currentConfiguration();
  return true;
}

auto PathPlanningInterface::setBound() -> bool {
  XmlRpc::XmlRpcValue x_lower;
  if (!getParam(nh_, pnh_, "x_lower", x_lower)) {
    return false;
  }
  XmlRpc::XmlRpcValue x_upper;
  if (!getParam(nh_, pnh_, "x_upper", x_upper)) {
    return false;
  }
  XmlRpc::XmlRpcValue y_lower;
  if (!getParam(nh_, pnh_, "y_lower", y_lower)) {
    return false;
  }
  XmlRpc::XmlRpcValue y_upper;
  if (!getParam(nh_, pnh_, "y_upper", y_upper)) {
    return false;
  }
  robot_->rootJoint()->lowerBound(0, x_lower);
  robot_->rootJoint()->upperBound(0, x_upper);
  robot_->rootJoint()->lowerBound(1, y_lower);
  robot_->rootJoint()->upperBound(1, y_upper);
  return true;
}

auto PathPlanningInterface::createObstacle() -> bool {
  XmlRpc::XmlRpcValue obstacle_name;
  if (!getParam(nh_, pnh_, "obstacle_name", obstacle_name)) {
    return false;
  }
  XmlRpc::XmlRpcValue obstacle_pkg_name;
  if (!getParam(nh_, pnh_, "obstacle_pkg_name", obstacle_pkg_name)) {
    return false;
  }
  XmlRpc::XmlRpcValue obstacle_model_name;
  if (!getParam(nh_, pnh_, "obstacle_model_name", obstacle_model_name)) {
    return false;
  }

  obstacle_ = Device::create(obstacle_name);
  hpp_pin::urdf::loadUrdfModel(obstacle_, "anchor", obstacle_pkg_name, obstacle_model_name);
  obstacle_->controlComputation(JOINT_POSITION);
  path_planning_solver_->addObstacle(obstacle_, true, true);
  return true;
}

auto PathPlanningInterface::setJointConfig(const sensor_msgs::JointState& msg,
                                           hpp_core::Configuration_t& config,
                                           const bool& update_names) -> size_t {
  if (update_names) {
    joint_names_.clear();
  }
  size_t valid = 0;
  for (size_t idx = 0; idx < msg.name.size(); ++idx) {
    try {
      auto joint = robot_->getJointByName(msg.name[idx]);
      config[joint->rankInConfiguration()] = msg.position[idx];
      if (update_names) {
        joint_names_.insert({msg.name[idx], std::make_pair(joint->rankInConfiguration(), joint->rankInVelocity())});
      }
      valid += 1;
    } catch (const std::runtime_error& e) {
      continue;
    }
  }
  return valid;
}

auto PathPlanningInterface::setLocationConfig(const geometry_msgs::Pose& msg, const int& type, Configuration_t& config)
    -> bool {
  if (!isPoseLegal(msg)) {
    return false;
  }

  if (type == 0) {
    Eigen::Quaterniond quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    Eigen::Rotation2Dd rot(quat.toRotationMatrix().topLeftCorner<2, 2>());
    double theta_cmd = rot.smallestPositiveAngle();
    config.head<kRootJointConfigDim>() << msg.position.x, msg.position.y, std::cos(theta_cmd), std::sin(theta_cmd);
    return true;
  }
  if (type == 1) {
    geometry_msgs::Pose global_to_local_aligned;
    global_to_local_aligned.position.x = q_current_[0];
    global_to_local_aligned.position.y = q_current_[1];
    global_to_local_aligned.orientation.w = 1.;

    geometry_msgs::Pose global_to_target;
    localAlignedPoseToGlobalPose(msg, global_to_local_aligned, global_to_target);
    return setLocationConfig(global_to_target, 0, config);
  }
  if (type == 2) {
    double theta_curr = atan2(q_current_[3], q_current_[2]);  // theta = atan2(sin(theta), cos(theta)) in (-pi, pi]
    Eigen::Quaterniond quat_curr = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(theta_curr, Eigen::Vector3d::UnitZ());
    geometry_msgs::Pose global_to_local;
    global_to_local.position.x = q_current_[0];
    global_to_local.position.y = q_current_[1];
    global_to_local.orientation.x = 0.;
    global_to_local.orientation.y = 0.;
    global_to_local.orientation.z = quat_curr.z();
    global_to_local.orientation.w = quat_curr.w();

    geometry_msgs::Pose global_to_target;
    localPoseToGlobalPose(msg, global_to_local, global_to_target);
    return setLocationConfig(global_to_target, 0, config);
  }
  return false;
}

void PathPlanningInterface::currentJointConfigCb(const sensor_msgs::JointState::ConstPtr& msg) {
  if (!is_initial_state_set_) {
    setJointConfig(*msg, q_current_, true);
    is_initial_state_set_ = true;
  } else {
    setJointConfig(*msg, q_current_, false);
  }
}

void PathPlanningInterface::currentLocationCb(const nav_msgs::Odometry::ConstPtr& msg) {
  setLocationConfig(msg->pose.pose, 0, q_current_);
  is_initial_location_set_ = true;
}

auto PathPlanningInterface::detectCollision(const Configuration_t& config) -> bool {
  DevicePtr_t dummy_robot = Device::createCopy(robot_);
  dummy_robot->currentConfiguration(config);
  dummy_robot->computeForwardKinematics();
  return dummy_robot->collisionTest();
}

void PathPlanningInterface::resetInitialConfig() {
  is_initial_state_set_ = false;
  is_initial_location_set_ = false;
}

auto PathPlanningInterface::executePathPlanningSrvCb(roport::ExecutePathPlanning::Request& req,
                                                     roport::ExecutePathPlanning::Response& resp) -> bool {
  if (is_initial_location_set_ && is_initial_state_set_) {
    ROS_INFO("Path planning started ...");
  } else {
    int count = kInitializeTimes;
    while (count > 0) {
      if (is_initial_location_set_ && is_initial_state_set_) {
        break;
      }
      ros::spinOnce();
      count--;
      ros::Duration(kInitializeInterval).sleep();
    }
    if (!is_initial_state_set_ || !is_initial_location_set_) {
      ROS_WARN("Initial state not set (location: %i, joint state: %i)", is_initial_location_set_,
               is_initial_state_set_);
      resp.result_status = resp.FAILED;
      return false;
    }
  }

  q_goal_ = q_current_;
  setLocationConfig(req.base_goal_pose, req.base_goal_type, q_goal_);
  setJointConfig(req.joint_goal_state, q_goal_, false);
  position_tolerance_ = req.base_pos_tolerance;
  orientation_tolerance_ = req.base_ori_tolerance;

  if (detectCollision(q_goal_)) {
    ROS_WARN("Goal state is in collision, aborted");
    resp.result_status = resp.FAILED;
    return false;
  }
  path_planning_solver_->addGoalConfig(std::make_shared<Configuration_t>(q_goal_));

  do {
    if (detectCollision(q_current_)) {
      ROS_WARN("Initial state is in collision, aborted");
      resp.result_status = resp.FAILED;
      return false;
    }
    ROS_INFO_STREAM("Curr: " << q_current_[0] << " " << q_current_[1] << " " << q_current_[2] << " " << q_current_[3]);
    ROS_INFO_STREAM("Goal: " << q_goal_[0] << " " << q_goal_[1] << " " << q_goal_[2] << " " << q_goal_[3]);
    path_planning_solver_->initConfig(std::make_shared<Configuration_t>(q_current_));
    path_planning_solver_->solve();

    // Get the last path from paths, which is an optimized one
    PathPtr_t optimized_path(path_planning_solver_->paths().back());
    value_type path_length(optimized_path->length());

    bool success;
    Configuration_t j_q;
    // j_dq 's size equals the dof
    vector_t j_dq = vector_t(optimized_path->outputDerivativeSize());

    int intervals = static_cast<int>(path_length / time_step_);
    ROS_INFO("Planning finished. Path time duration: %f, waypoints #%i, Sending command ...", path_length, intervals);

    std::vector<sensor_msgs::JointState> joint_states;
    std::vector<geometry_msgs::Twist> vel_cmd;
    for (int i = 0; i <= intervals; i += 1) {
      double stamp = i * time_step_;
      j_q = optimized_path->eval(stamp, success);
      optimized_path->derivative(j_dq, stamp, 1);
      if (!success) {
        return false;
      }
      sensor_msgs::JointState state;
      extractJointCommand(j_q, j_dq, state);
      joint_states.push_back(state);

      geometry_msgs::Twist twist;
      extractBaseVelocityCommand(j_q, j_dq, twist);
      vel_cmd.push_back(twist);
    }

    publishPlanningResults(joint_states, vel_cmd);

    resetInitialConfig();
  } while (!checkGoalLocationReached());

  // Necessary for using the last goal
  path_planning_solver_->resetGoalConfigs();
  ROS_INFO("Path execution finished.");
  return true;
}

void PathPlanningInterface::extractJointCommand(const Configuration_t& j_q,
                                                const vector_t& j_dq,
                                                sensor_msgs::JointState& state) {
  for (auto& element : joint_names_) {
    state.name.push_back(element.first);
    state.position.push_back(j_q[element.second.first]);
    state.velocity.push_back(j_dq[element.second.second]);
    state.effort.push_back(0);
  }
}

void PathPlanningInterface::extractBaseVelocityCommand(const Configuration_t& j_q,
                                                       const vector_t& j_dq,
                                                       geometry_msgs::Twist& twist) {
  Eigen::Rotation2Dd rot(atan2(j_q[3], j_q[2]));
  Eigen::Vector2d global_linear(j_dq[0], j_dq[1]);
  Eigen::Vector2d local_linear = rot.inverse() * global_linear;

  twist.linear.x = local_linear.x() * kReductionRatio;
  twist.linear.y = local_linear.y() * kReductionRatio;
  twist.angular.z = j_dq[2] * kReductionRatio;
}

void PathPlanningInterface::publishPlanningResults(const std::vector<sensor_msgs::JointState>& joint_states,
                                                   const std::vector<geometry_msgs::Twist>& vel_cmd) {
  // Since all vel_cmd are the same, we only use the first
  base_vel_cmd_publisher_.publish(vel_cmd[0]);
  geometry_msgs::Twist zero;

  ros::Duration duration(time_step_ / kReductionRatio);
  bool goal_location_reached = false;
  for (const auto& joint_state : joint_states) {
    joint_command_publisher_.publish(joint_state);
    if (!goal_location_reached) {
      if (checkGoalLocationReached()) {
        // Early stop
        base_vel_cmd_publisher_.publish(zero);
        goal_location_reached = true;
      }
    }
    duration.sleep();
  }
  // Stop the movement
  base_vel_cmd_publisher_.publish(zero);
}

auto PathPlanningInterface::checkGoalLocationReached() -> bool {
  std::vector<double> goal_location;
  for (int i = 0; i < kRootJointConfigDim; ++i) {
    goal_location.push_back(q_goal_[i]);
  }
  std::vector<double> current_location;
  for (int i = 0; i < kRootJointConfigDim; ++i) {
    current_location.push_back(q_current_[i]);
  }
  size_t violated_i;
  double error;
  std::vector<double> tol;
  tol.resize(current_location.size());
  std::fill(tol.begin(), tol.begin() + root_joint_.getPositionConfigDim(root_joint_type_), position_tolerance_);
  std::fill(tol.begin() + root_joint_.getPositionConfigDim(root_joint_type_), tol.end(), orientation_tolerance_);
  return allClose<double>(current_location, goal_location, violated_i, error, tol);
}
}  // namespace roport
