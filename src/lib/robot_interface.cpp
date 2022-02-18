#include "roport/robot_interface.h"

namespace roport {
RobotInterface::RobotInterface(ros::NodeHandle& node_handle, ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh), received_first_state_(false), received_num_(0), num_joints_(0) {
  init();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(this, nh_);
  const double kInterval = 0.001;
  non_realtime_loop_ = nh_.createTimer(ros::Duration(kInterval), &RobotInterface::update, this);
  ROS_INFO("Robot hardware interface ready.");
}

RobotInterface::~RobotInterface() = default;

void RobotInterface::init() {
  // This param is a global one, use nh_ to get it
  std::string joint_name_param_id;
  nh_.param<std::string>("joint_name_param_id", joint_name_param_id, "/hardware_interface/joints");

  std::vector<std::string> joint_names;
  if (!nh_.getParam(joint_name_param_id, joint_names) || joint_names.empty()) {
    throw std::runtime_error("Got no valid joint name from " + joint_name_param_id);
  }

  joint_names_.insert(joint_names_.end(), joint_names.begin(), joint_names.end());
  for (const auto& name : joint_names_) {
    ROS_INFO_STREAM("Initialized joint " << name);
  }
  num_joints_ = joint_names_.size();

  // The topic of the joint states measured by the robot encoder or provided by a simulator
  XmlRpc::XmlRpcValue raw_measured_joint_states_id;
  pnh_.getParam("measure_joint_states_id", raw_measured_joint_states_id);
  ROS_ASSERT(raw_measured_joint_states_id.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (raw_measured_joint_states_id.size() == 0) {
    throw std::runtime_error("Raw measured joint state id is None");
  }
  // For each measured joint state message, we initialize a subscriber with a callback function
  // knowing which topic it will subscribe to, the topic name is recorded in a global temp for
  // determining if every initial state has been received.
  for (int i = 0; i < raw_measured_joint_states_id.size(); i++) {
    ROS_ASSERT(raw_measured_joint_states_id[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string measured_topic = std::string(raw_measured_joint_states_id[i]);
    ros::Subscriber js_getter = nh_.subscribe<sensor_msgs::JointState>(
        measured_topic, 1, [this, measured_topic](auto && ph1) { jointStatesCb(std::forward<decltype(ph1)>(ph1), measured_topic); });
    js_getters_.insert({measured_topic, js_getter});
    ROS_INFO_STREAM("Get measured joint states from <- " << measured_topic);
  }

  // The topic of the joint states published by this interface and received by real robot or simulator
  XmlRpc::XmlRpcValue control_joint_states_id;
  pnh_.getParam("control_joint_states_id", control_joint_states_id);
  ROS_ASSERT(control_joint_states_id.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (control_joint_states_id.size() == 0) {
    throw std::runtime_error("Control joint state command topic id is None");
  }
  for (int i = 0; i < control_joint_states_id.size(); i++) {
    ROS_ASSERT(control_joint_states_id[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    ros::Publisher js_setter = nh_.advertise<sensor_msgs::JointState>(control_joint_states_id[i], 1);
    js_setters_.push_back(js_setter);
    ROS_INFO_STREAM("Publishing joint states commands -> " << control_joint_states_id[i]);
  }

  // The joint names in the joint groups. If no group is provided, all joint names are in the same group
  XmlRpc::XmlRpcValue joint_states_group;
  pnh_.getParam("joint_states_group", joint_states_group);
  if (joint_states_group.size() == 0) {
    active_joint_groups_.push_back(joint_names_);
    ROS_INFO_STREAM("Using all joints in a single group");
  } else {
    for (int i = 0; i < joint_states_group.size(); i++) {
      ROS_ASSERT(joint_states_group[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::vector<std::string> active_joint_group;
      active_joint_group.reserve(joint_states_group[i].size());
      for (int j = 0; j < joint_states_group[i].size(); ++j) {
        active_joint_group.push_back(joint_states_group[i][j]);
      }
      active_joint_groups_.push_back(active_joint_group);
      ROS_INFO_STREAM("Joint group " << i << " size: " << active_joint_group.size());
    }
  }

  // One getter and one setter for one group
  ROS_ASSERT(js_getters_.size() == js_setters_.size() && js_setters_.size() == active_joint_groups_.size());

  // Resize state vectors
  q_.resize(num_joints_);
  dq_.resize(num_joints_);
  tau_j_.resize(num_joints_);

  std::fill(q_.begin(), q_.end(), 0.0);
  std::fill(dq_.begin(), dq_.end(), 0.0);
  std::fill(tau_j_.begin(), tau_j_.end(), 0.0);

  XmlRpc::XmlRpcValue joint_init_positions;  // The initial positions of the joints
  XmlRpc::XmlRpcValue joint_init_names;      // Joints that need initial position values
  std::vector<std::string> init_names;
  std::vector<double> init_positions;

  // If predefined joint initial positions are given, set the initial configuration of the robot
  // with these values, otherwise, use the current (subject to starting this node) configuration
  // as the initial configuration.
  if (pnh_.getParam("joint_init_positions", joint_init_positions) &&
      pnh_.getParam("joint_init_names", joint_init_names)) {
    ROS_ASSERT(joint_init_positions.size() == joint_init_names.size());
    for (int i = 0; i < joint_init_positions.size(); ++i) {
      ROS_ASSERT(joint_init_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      init_names.push_back(joint_init_names[i]);
      double init_position;
      if (joint_init_positions[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        init_position = joint_init_positions[i];
      } else if (joint_init_positions[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        int tmp = joint_init_positions[i];
        init_position = static_cast<double>(tmp);
      } else {
        std::string error_msg = "Unsupported position type: " + std::to_string(joint_init_positions[i].getType());
        throw std::runtime_error(error_msg);
      }
      init_positions.push_back(init_position);
      ROS_INFO_STREAM("Set " << joint_init_names[i] << " initial position as " << joint_init_positions[i]);
    }
  } else {
    while (!received_first_state_) {
      ros::spinOnce();
    }
    ros::Time now = ros::Time::now();
    ros::Duration duration;
    read(now, duration);
    for (size_t i = 0; i < num_joints_; ++i) {
      init_names.push_back(joint_names_[i]);
      init_positions.push_back(q_[i]);
    }
  }

  // Initialize joint state command vector
  q_cmd_.resize(num_joints_, 0.0);
  dq_cmd_.resize(num_joints_, 0.0);
  tau_j_cmd_.resize(num_joints_, 0.0);

  // Set the joint positions according to the initial positions
  for (int i = 0; i < init_names.size(); ++i) {
    for (int j = 0; j < num_joints_; ++j) {
      if (init_names[i] == joint_names_[j]) {
        q_cmd_[j] = init_positions[i];
      }
    }
  }

  // Initialize interface for each joint
  for (int i = 0; i < num_joints_; ++i) {
    // Create joint state update interface
    JointStateHandle joint_state_handle(joint_names_[i], &q_[i], &dq_[i], &tau_j_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Add joint control interfaces
    JointHandle joint_position_handle(joint_state_handle, &q_cmd_[i]);
    q_cmd_interface_.registerHandle(joint_position_handle);
    JointHandle joint_velocity_handle(joint_state_handle, &dq_cmd_[i]);
    dq_cmd_interface_.registerHandle(joint_velocity_handle);
    JointHandle joint_effort_handle(joint_state_handle, &tau_j_cmd_[i]);
    tau_j_cmd_interface_.registerHandle(joint_effort_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&q_cmd_interface_);
  registerInterface(&dq_cmd_interface_);
  registerInterface(&tau_j_cmd_interface_);
  ROS_INFO_STREAM("Interfaces initialized");
}

void RobotInterface::update(const ros::TimerEvent& event) {
  ros::Time now = ros::Time::now();
  if (event.current_real < event.last_real) {
    ROS_ERROR("Time jump detected, you should only see this when use_sim_time=true, need rerun.");
    return;
  }
  auto elapsed_duration = ros::Duration(event.current_real - event.last_real);
  read(now, elapsed_duration);
  ROS_DEBUG("Time for now: %f, elapsed: %f|", now.toSec(), elapsed_duration.toSec());
  controller_manager_->update(now, elapsed_duration);
  write(now, elapsed_duration);
}

void RobotInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (int i = 0; i < num_joints_; i++) {
    try {
      std::unique_lock<std::timed_mutex> lock(joint_states_mutex_, std::chrono::seconds(2));
      if (lock) {
        auto search = joint_states_.find(joint_names_[i]);
        if (search != joint_states_.end()) {
          std::vector<double> joint_state = search->second;
          q_.at(i) = joint_state[0];
          dq_.at(i) = joint_state[1];
          tau_j_.at(i) = joint_state[2];
        }
      } else {
        ROS_ERROR("Couldn't acquire mutex, couldn't get joint state");
      }
    } catch (std::out_of_range& e) {
      ROS_WARN_STREAM("Failed to get joint " << joint_names_[i] << " state, no message received yet");
    }
  }
}

void RobotInterface::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (int i = 0; i < js_setters_.size(); ++i) {
    sendJointPositionCmd(js_setters_[i], active_joint_groups_[i]);
  }
}

void RobotInterface::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg, const std::string& topic) {
  std::unique_lock<std::timed_mutex> lock(joint_states_mutex_, std::chrono::seconds(2));
  if (!lock) {
    ROS_ERROR("Couldn't acquire mutex, couldn't set joint state");
    return;
  }
  try {
    // Make sure velocity and effort are not empty in hardware or simulator API output
    ROS_ASSERT(!msg->name.empty() && msg->name.size() == msg->position.size());
    for (int i = 0; i < msg->name.size(); i++) {
      std::vector<double> pve;
      if (msg->position.empty()) {
        throw std::runtime_error("Robot joint position cannot be empty");
      }
      pve.push_back(msg->position.at(i));

      if (msg->velocity.empty()) {
        pve.push_back(0.);
      } else {
        pve.push_back(msg->velocity.at(i));
      }
      if (msg->effort.empty()) {
        pve.push_back(0.);
      } else {
        pve.push_back(msg->effort.at(i));
      }
      joint_states_[msg->name[i]] = pve;
    }
    if (!received_first_state_) {
      for (std::map<std::string, ros::Subscriber>::iterator it = js_getters_.begin(); it != js_getters_.end(); ++it) {
        std::string key = it->first;
        if (key == topic) {
          received_num_++;
        }
      }
      ROS_WARN_STREAM_THROTTLE(3, "Receiving the init joint state... " << received_num_ << "/" << js_getters_.size());
      if (received_num_ >= js_getters_.size()) {
        received_first_state_ = true;
      }
    }
  } catch (const std::out_of_range& oor) {
    ROS_ERROR_STREAM("Out of Range error: " << oor.what());
  }
}

void RobotInterface::sendJointPositionCmd(const ros::Publisher& setter, const std::vector<std::string>& js_names) {
  sensor_msgs::JointState msg;
  std::vector<double> js_q;
  js_q.reserve(js_names.size());
  for (const auto& js_name : js_names) {
    for (int i = 0; i < num_joints_; ++i) {
      if (js_name == joint_names_[i]) {
        js_q.push_back(q_cmd_[i]);
      }
    }
  }
  msg.header.frame_id = setter.getTopic();
  msg.name = js_names;
  msg.position = js_q;
  setter.publish(msg);
}
}  // namespace roport
