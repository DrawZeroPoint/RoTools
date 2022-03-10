#include <memory>

#include "roport/hpp_interface.h"

namespace roport {
HumanoidPathPlannerInterface::HumanoidPathPlannerInterface(const ros::NodeHandle& node_handle,
                                                           const ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh), is_initial_state_set_(false), is_initial_location_set_(false), num_joints_(0) {
  solver_ = ProblemSolver::create();
  if (!createRobot()) {
    return;
  }
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
    loaded = plugin::loadPlugin(filename, solver_);
  } catch (const std::invalid_argument&) {
    loaded = false;
  }
  if (loaded) {
    ROS_INFO("Using path optimizer: SplineGradientBased_bezier1");
    solver_->addPathOptimizer("SplineGradientBased_bezier1");
  } else {
    ROS_WARN("Could not load spline-gradient-based.so, using path optimizer: RandomShortcut");
    solver_->addPathOptimizer("RandomShortcut");
  }

  XmlRpc::XmlRpcValue state_topic_id;
  if (!getParam(nh_, pnh_, "state_topic_id", state_topic_id)) {
    return;
  }
  state_subscriber_ = nh_.subscribe<sensor_msgs::JointState>(
      state_topic_id, 1, [this](auto&& ph1) { return initialJointStateCb(std::forward<decltype(ph1)>(ph1)); });

  XmlRpc::XmlRpcValue location_topic_id;
  if (!getParam(nh_, pnh_, "location_topic_id", location_topic_id)) {
    return;
  }
  location_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(
      location_topic_id, 1, [this](auto&& ph1) { return initialLocationCb(std::forward<decltype(ph1)>(ph1)); });

  XmlRpc::XmlRpcValue set_initial_srv_id;
  if (!getParam(nh_, pnh_, "execute_reinitialize_planning_srv_id", set_initial_srv_id)) {
    return;
  }
  set_initial_srv_ = nh_.advertiseService(set_initial_srv_id, &HumanoidPathPlannerInterface::setInitialSrvCb, this);

  XmlRpc::XmlRpcValue execute_path_planning_srv_id;
  if (!getParam(nh_, pnh_, "execute_path_planning_srv_id", execute_path_planning_srv_id)) {
    return;
  }
  execute_path_planning_srv_ =
      nh_.advertiseService(execute_path_planning_srv_id, &HumanoidPathPlannerInterface::executePathPlanningSrvCb, this);

  ROS_INFO("Robot HPP interface ready");
}

bool HumanoidPathPlannerInterface::createRobot() {
  XmlRpc::XmlRpcValue robot_name;
  if (!getParam(nh_, pnh_, "robot_name", robot_name)) {
    return false;
  }
  XmlRpc::XmlRpcValue robot_pkg_name;
  if (!getParam(nh_, pnh_, "robot_pkg_name", robot_pkg_name)) {
    return false;
  }
  XmlRpc::XmlRpcValue model_name;
  if (!getParam(nh_, pnh_, "model_name", model_name)) {
    return false;
  }

  robot_ = hpp::pinocchio::Device::create(std::string(robot_name));
  // Only support planar root
  hpp::pinocchio::urdf::loadRobotModel(robot_, "planar", robot_pkg_name, model_name, "", "");
  robot_->controlComputation((Computation_t)(JOINT_POSITION | JACOBIAN));
  solver_->robot(robot_);
  q_init_ = robot_->currentConfiguration();
  return true;
}

bool HumanoidPathPlannerInterface::setBound() {
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

bool HumanoidPathPlannerInterface::createObstacle() {
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
  hpp::pinocchio::urdf::loadUrdfModel(obstacle_, "anchor", obstacle_pkg_name, obstacle_model_name);
  obstacle_->controlComputation(JOINT_POSITION);
  solver_->addObstacle(obstacle_, true, true);
  return true;
}

bool HumanoidPathPlannerInterface::setJointConfig(const std::string& joint_name,
                                                  const double& joint_value,
                                                  Configuration_t& config) {
  try {
    auto joint = robot_->getJointByName(joint_name);
    config[joint->rankInConfiguration()] = joint_value;
    return true;
  } catch (const std::runtime_error& e) {
    return false;
  }
}

void HumanoidPathPlannerInterface::setLocation(const nav_msgs::Odometry::ConstPtr& msg, Configuration_t& config) {
  config.head<6>() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
}

bool HumanoidPathPlannerInterface::setLocation(const geometry_msgs::Pose& msg,
                                               const int& type,
                                               Configuration_t& config) {
  if (type == 0) {
    config.head<6>() << msg.position.x, msg.position.y, msg.orientation.w, msg.orientation.x, msg.orientation.y,
        msg.orientation.z;
    return true;
  } else if (type == 1) {
    ROS_WARN("Not implemented");
    return false;
  } else if (type == 2) {
    ROS_WARN("Not implemented");
    return false;
  } else {
    return false;
  }
}

void HumanoidPathPlannerInterface::initialJointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
  if (is_initial_state_set_) {
    return;
  }
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (!setJointConfig(msg->name[i], msg->position[i], q_init_)) {
      continue;
    }
  }
  ROS_INFO("Initial joint state updated");
  is_initial_state_set_ = true;
}

void HumanoidPathPlannerInterface::initialLocationCb(const nav_msgs::Odometry::ConstPtr& msg) {
  if (is_initial_location_set_) {
    return;
  }
  setLocation(msg, q_init_);
  ROS_INFO("Initial location updated");
  is_initial_location_set_ = true;
}

auto HumanoidPathPlannerInterface::setInitialSrvCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
    -> bool {
  is_initial_state_set_ = false;
  is_initial_location_set_ = false;
  return true;
}

auto HumanoidPathPlannerInterface::executePathPlanningSrvCb(roport::ExecutePathPlanning::Request& req,
                                                            roport::ExecutePathPlanning::Response& resp) -> bool {
  if (is_initial_location_set_ && is_initial_state_set_) {
    ROS_INFO("Path planning started ...");
    std::cout << "init (" << displayConfig(q_init_, 4) << ")" << std::endl;
    std::cout << robot_->getJointByName("head_actuated_joint1")->rankInConfiguration() << std::endl;
  } else {
    ROS_WARN("Initial state not set (location: %i, joint state: %i)", is_initial_location_set_, is_initial_state_set_);
    resp.result_status = resp.FAILED;
    return false;
  }
  solver_->initConfig(std::make_shared<Configuration_t>(q_init_));

  Configuration_t q_goal(q_init_);
//  if (!setLocation(req.goal_location, req.goal_type, q_goal)) {
    q_goal.head<2>() << -3.5, -4;
    q_goal[robot_->getJointByName("panda_left_joint1")->rankInConfiguration()] = 1;
    q_goal[robot_->getJointByName("panda_left_joint2")->rankInConfiguration()] = 1;
    q_goal[robot_->getJointByName("panda_left_joint4")->rankInConfiguration()] = -1.5;
    q_goal[robot_->getJointByName("panda_right_joint1")->rankInConfiguration()] = -1;
    q_goal[robot_->getJointByName("panda_right_joint2")->rankInConfiguration()] = 1;
    q_goal[robot_->getJointByName("panda_right_joint4")->rankInConfiguration()] = -1.5;
//  }

  solver_->addGoalConfig(std::make_shared<Configuration_t>(q_goal));
  solver_->solve();

  // Get the last optimized_path, which is an optimized one
  PathPtr_t optimized_path(solver_->paths().back());
  value_type path_length(optimized_path->length());

  bool success;
  Configuration_t q;
  vector_t dq = vector_t(optimized_path->outputDerivativeSize());

  std::vector<double> joint_state;
  for (value_type t = 0; t < path_length; t += .01) {
    q = optimized_path->eval(t, success);
    optimized_path->derivative(dq, t, 1);
    if (!success) {
      return false;
    }

    for (size_type i = 0; i < q.size(); ++i) {
      if (i <= 5) {
        continue;
      }
      joint_state.push_back(q[i]);
    }
    std::cout << dq.size() << std::endl;
    std::cout << "optimized_path.append (" << displayConfig(q, 6) << ")" << std::endl;
  }
  q = optimized_path->eval(path_length, success);
  std::cout << "optimized_path.append (" << displayConfig(q, 6) << ")" << std::endl;
  return true;
}

}  // namespace roport
