#include <memory>

#include "roport/hpp_interface.h"

namespace roport {
HumanoidPathPlannerInterface::HumanoidPathPlannerInterface(const ros::NodeHandle& node_handle,
                                                           const ros::NodeHandle& pnh)
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
    return;
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
  initial_state_subscriber_ = nh_.subscribe<sensor_msgs::JointState>(
      state_topic_id, 1, [this](auto&& ph1) { return initialJointConfigCb(std::forward<decltype(ph1)>(ph1)); });

  current_state_subscriber_ = nh_.subscribe<sensor_msgs::JointState>(
      state_topic_id, 1, [this](auto&& ph1) { return currentJointConfigCb(std::forward<decltype(ph1)>(ph1)); });

  XmlRpc::XmlRpcValue location_topic_id;
  if (!getParam(nh_, pnh_, "location_topic_id", location_topic_id)) {
    return;
  }
  initial_location_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      location_topic_id, 1, [this](auto&& ph1) { return initialLocationCb(std::forward<decltype(ph1)>(ph1)); });

  current_location_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      location_topic_id, 1, [this](auto&& ph1) { return currentLocationCb(std::forward<decltype(ph1)>(ph1)); });

  XmlRpc::XmlRpcValue execute_path_planning_srv_id;
  if (!getParam(nh_, pnh_, "execute_path_planning_srv_id", execute_path_planning_srv_id)) {
    return;
  }
  execute_path_planning_srv_ =
      nh_.advertiseService(execute_path_planning_srv_id, &HumanoidPathPlannerInterface::executePathPlanningSrvCb, this);

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

auto HumanoidPathPlannerInterface::createRobot() -> bool {
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
  q_init_ = robot_->currentConfiguration();
  q_current_ = robot_->currentConfiguration();
  return true;
}

auto HumanoidPathPlannerInterface::setBound() -> bool {
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

auto HumanoidPathPlannerInterface::createObstacle() -> bool {
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

auto HumanoidPathPlannerInterface::setJointConfig(const sensor_msgs::JointState& msg,
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

auto HumanoidPathPlannerInterface::setLocationConfig(const geometry_msgs::Pose& msg,
                                                     const int& type,
                                                     Configuration_t& config) -> bool {
  if (!isPoseLegal(msg)) {
    return false;
  }
  Eigen::Quaterniond quat_curr(config[2], 0, 0, config[3]);
  Eigen::Quaterniond quat_cmd(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  auto angles = quat_cmd.toRotationMatrix().eulerAngles(2, 1, 0);
  auto yaw = angles[0] * 2;
  if (yaw > M_PI) {
    yaw = yaw - M_PI * 2.;
  }
  Eigen::Quaterniond quat_new = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  if (quat_curr.dot(quat_new) < 0.) {
    quat_new = quat_new.conjugate();
  }
  if (type == 0) {
    ROS_ASSERT(kRootJointConfigDim == root_joint_.getConfigDim(root_joint_type_));
    config.head<kRootJointConfigDim>() << msg.position.x, msg.position.y, quat_new.w(), quat_new.z();
    return true;
  }
  if (type == 1) {
    ROS_ASSERT(kRootJointPositionConfigDim == root_joint_.getPositionConfigDim(root_joint_type_));
    config.head<kRootJointPositionConfigDim>() << config[0] + msg.position.x, config[1] + msg.position.y;
    return true;
  }
  if (type == 2) {
    geometry_msgs::Pose global_to_local;
    global_to_local.position.x = config[0];
    global_to_local.position.y = config[1];
    global_to_local.orientation.w = config[2];
    global_to_local.orientation.x = 0.;
    global_to_local.orientation.y = 0.;
    global_to_local.orientation.z = config[3];

    geometry_msgs::Pose global_to_target;
    localPoseToGlobalPose(msg, global_to_local, global_to_target);
    return setLocationConfig(global_to_target, 0, config);
  }
  return false;
}

void HumanoidPathPlannerInterface::initialJointConfigCb(const sensor_msgs::JointState::ConstPtr& msg) {
  if (is_initial_state_set_) {
    return;
  }
  std::lock_guard<std::mutex> lock(init_mutex_);
  size_t valid = setJointConfig(*msg, q_init_, true);
  is_initial_state_set_ = true;
  ROS_INFO("Initial %lu joint state updated. Robot dof (except base): %ld", valid,
           robot_->numberDof() - root_joint_.getDOF(root_joint_type_));
}

void HumanoidPathPlannerInterface::currentJointConfigCb(const sensor_msgs::JointState::ConstPtr& msg) {
  setJointConfig(*msg, q_current_, false);
}

void HumanoidPathPlannerInterface::initialLocationCb(const nav_msgs::Odometry::ConstPtr& msg) {
  if (is_initial_location_set_) {
    return;
  }
  std::lock_guard<std::mutex> lock(init_mutex_);
  // The pose from odom msg is considered as global
  setLocationConfig(msg->pose.pose, 0, q_init_);
  is_initial_location_set_ = true;
  ROS_INFO("Initial location updated");
}

void HumanoidPathPlannerInterface::currentLocationCb(const nav_msgs::Odometry::ConstPtr& msg) {
  setLocationConfig(msg->pose.pose, 0, q_current_);
}

auto HumanoidPathPlannerInterface::detectCollision(const Configuration_t& config) -> bool {
  DevicePtr_t dummy_robot = Device::createCopy(robot_);
  dummy_robot->currentConfiguration(config);
  dummy_robot->computeForwardKinematics();
  return dummy_robot->collisionTest();
}

void HumanoidPathPlannerInterface::resetInitialConfig() {
  is_initial_state_set_ = false;
  is_initial_location_set_ = false;
}

auto HumanoidPathPlannerInterface::executePathPlanningSrvCb(roport::ExecutePathPlanning::Request& req,
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

  q_goal_ = q_init_;
  setLocationConfig(req.goal_location, req.goal_type, q_goal_);
  setJointConfig(req.goal_state, q_goal_, false);
  position_tolerance_ = req.pos_tolerance;
  orientation_tolerance_ = req.ori_tolerance;

  if (detectCollision(q_goal_)) {
    ROS_WARN("Goal state is in collision, aborted");
    resp.result_status = resp.FAILED;
    return false;
  }
  path_planning_solver_->addGoalConfig(std::make_shared<Configuration_t>(q_goal_));

  do {
    if (detectCollision(q_init_)) {
      ROS_WARN("Initial state is in collision, aborted");
      resp.result_status = resp.FAILED;
      return false;
    }
    path_planning_solver_->initConfig(std::make_shared<Configuration_t>(q_init_));

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
      extractBaseVelocityCommand(j_dq, twist);
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

void HumanoidPathPlannerInterface::extractJointCommand(const Configuration_t& j_q,
                                                       const vector_t& j_dq,
                                                       sensor_msgs::JointState& state) {
  for (auto& element : joint_names_) {
    state.name.push_back(element.first);
    state.position.push_back(j_q[element.second.first]);
    state.velocity.push_back(j_dq[element.second.second]);
    state.effort.push_back(0);
  }
}

void HumanoidPathPlannerInterface::extractBaseVelocityCommand(const vector_t& j_dq, geometry_msgs::Twist& twist) {
  twist.linear.x = j_dq[0] * kReductionRatio;
  twist.linear.y = j_dq[1] * kReductionRatio;
  twist.angular.z = j_dq[2] * kReductionRatio;
}

void HumanoidPathPlannerInterface::publishPlanningResults(const std::vector<sensor_msgs::JointState>& joint_states,
                                                          const std::vector<geometry_msgs::Twist>& vel_cmd) {
  // Since all vel_cmd are the same, we only use the first
  base_vel_cmd_publisher_.publish(vel_cmd[0]);
  geometry_msgs::Twist zero;

  ros::Duration duration(time_step_ / kReductionRatio);
  for (const auto& joint_state : joint_states) {
    joint_command_publisher_.publish(joint_state);
    if (checkGoalLocationReached()) {
      // Early stop
      base_vel_cmd_publisher_.publish(zero);
    }
    duration.sleep();
  }
  // Stop the movement
  base_vel_cmd_publisher_.publish(zero);
}

auto HumanoidPathPlannerInterface::checkGoalLocationReached() -> bool {
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
