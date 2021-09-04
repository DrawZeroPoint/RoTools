#include <ros/ros.h>
#include <ros/console.h>
#include <boost/bind.hpp>

#include <sensor_msgs/JointState.h>

#ifdef FRANKA_CORE_MSGS
#include <franka_core_msgs/JointCommand.h>
#endif

#ifdef YOUR_CUSTOM_MSGS
// Include your message headers
#endif

std::vector<ros::Publisher> publishers_;
std::vector<ros::Subscriber> subscribers_;

/**
 * Generic function to find an element in vector and also its position. It returns a pair of bool & int.
 * @tparam T
 * @param vecOfElements
 * @param element
 * @return bool: Represents if element is present in vector or not.
 *         int: Represents the index of element in vector if its found else -1
 */
template < typename T>
std::pair<bool, int> findInVector(const std::vector<T>& vecOfElements, const T& element)
{
  std::pair<bool, int > result;
  // Find given element in vector
  auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);
  if (it != vecOfElements.end()) {
    result.second = distance(vecOfElements.begin(), it);
    result.first = true;
  } else {
    result.first = false;
    result.second = -1;
  }
  return result;
}

void publishJointState(const sensor_msgs::JointState::ConstPtr& msg, const ros::Publisher &pub,
                       const std::vector<std::string> &source_names, const std::vector<std::string> &target_names) {
  sensor_msgs::JointState new_msg;
  new_msg.header = msg->header;

  if (msg->name.empty()) {
    ROS_ERROR_ONCE_NAMED("RoPort Converter", "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given message.
  if (msg->name.size() < source_names.size()) {
    ROS_ERROR_NAMED("RoPort Converter", "Message name field has fewer names than source names");
    return;
  }
  for (size_t i = 0; i < source_names.size(); ++i) {
    auto result = findInVector<std::string>(msg->name, source_names[i]);
    if (result.first) {
      new_msg.name.push_back(target_names[i]);
      new_msg.position.push_back(msg->position[result.second]);
      if (msg->velocity.size() == msg->position.size()) {
        new_msg.velocity.push_back(msg->velocity[result.second]);
      }
      if (msg->effort.size() == msg->effort.size()) {
        new_msg.effort.push_back(msg->effort[result.second]);
      }
    } else {
      ROS_ERROR_STREAM_NAMED("RoPort Converter", "Source joint name "
          << source_names[i] << "does not match any name in the given JointState message");
    }
  }
  pub.publish(new_msg);
}

void publishJointCommand(const sensor_msgs::JointState::ConstPtr& msg, const ros::Publisher &pub, const int& arg,
                         const std::vector<std::string> &source_names, const std::vector<std::string> &target_names) {
#ifdef FRANKA_CORE_MSGS
  franka_core_msgs::JointCommand new_msg;
  new_msg.header = msg->header;
  std::set<int> supported_modes{
    new_msg.IMPEDANCE_MODE, new_msg.POSITION_MODE, new_msg.TORQUE_MODE, new_msg.VELOCITY_MODE
  };
  if (supported_modes.find(arg) == supported_modes.end()) {
    ROS_ERROR_STREAM_ONCE_NAMED("RoPort Converter", "Mode " << arg << " is not supported");
    return;
  } else {
    new_msg.mode = arg;
  }
  if (msg->name.empty()) {
    ROS_ERROR_ONCE_NAMED("RoPort Converter", "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given message.
  if (msg->name.size() < source_names.size()) {
    ROS_ERROR_NAMED("RoPort Converter", "Message name field has fewer names than source names");
    return;
  }
  for (size_t i = 0; i < source_names.size(); ++i) {
    auto result = findInVector<std::string>(msg->name, source_names[i]);
    if (result.first) {
      new_msg.names.push_back(target_names[i]);
      new_msg.position.push_back(msg->position[result.second]);
      if (msg->velocity.size() == msg->position.size()) {
        new_msg.velocity.push_back(msg->velocity[result.second]);
      }
      if (msg->effort.size() == msg->effort.size()) {
        new_msg.effort.push_back(msg->effort[result.second]);
      }
    } else {
      ROS_ERROR_STREAM_NAMED("RoPort Converter", "Source joint name "
          << source_names[i] << "does not match any name in the given JointState message");
    }
  }
  pub.publish(new_msg);
#endif
}

void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg, const ros::Publisher &pub,
                  const std::string& type, const int& arg,
                  const std::vector<std::string> &source_names, const std::vector<std::string> &target_names) {
  if (type == "sensor_msgs/JointState") {
    publishJointState(msg, pub, source_names, target_names);
  } else if (type == "franka_core_msgs/JointCommand") {
    publishJointCommand(msg, pub, arg, source_names, target_names);
  } else {
    ROS_ERROR_STREAM_ONCE_NAMED("RoPort Converter", "Unknown target topic type: "
        << type << " Known are: sensor_msgs/JointState, franka_core_msgs/JointCommand");
  }
}

/**
 * Convert sensor_msgs::JointState from one topic to another, the name field could change during the convention
 * To use this function, in the launch file, we need define the following parameters:
 *   source_joint_names: list[list] The outer list is for all joint groups, each joint group is composed by several
 *                       joints that should be measured together and be controlled by a control topic.
 *   target_joint_names: list[list] This field have one-on-one correspondence with source_joint_names, the given joint
 *                       values will be under the new names defined by target_joint_names.
 *   source_js_topics: list[str] Joint state topics that should be converted to the target_js_topics.
 *   target_js_topics: list[str] Topics converted from source_js_topics.
 *   target_types: optional list[str] Target topic types. By default, the target topics will have the type JointState,
 *                 other types are also supported but need modifying the code. Here we support sensor_msgs/JointState
 *                 and franka_core_msgs/JointCommand. If given, its size must be equal to target_js_topics.
 *   target_args: optional list[int] Give one argument for each target topic publishing. This argument often
 *                defines the control type. If given, its size must be equal to target_js_topics.
 *
 * An example launch file should look like this:
 *
  <node pkg="roport" type="roport_msg_converter" name="curiosity_converter" output="screen">
    <rosparam param="source_joint_names">
      [
        ["arm_L_joint1", "arm_L_joint2", "arm_L_joint3", "arm_L_joint4", "arm_L_joint5", "arm_L_joint6", "arm_L_joint7"],
        ["arm_R_joint1", "arm_R_joint2", "arm_R_joint3", "arm_R_joint4", "arm_R_joint5", "arm_R_joint6", "arm_R_joint7"],
      ]
    </rosparam>
    <rosparam param="target_joint_names">
      [
        ["panda_left_joint1", "panda_left_joint2", "panda_left_joint3", "panda_left_joint4", "panda_left_joint5", "panda_left_joint6", "panda_left_joint7"],
        ["panda_right_joint1", "panda_right_joint2", "panda_right_joint3", "panda_right_joint4", "panda_right_joint5", "panda_right_joint6", "panda_right_joint7"],
      ]
    </rosparam>
    <rosparam param="source_js_topics">
      ["/cartesian/solution", "/cartesian/solution"]
    </rosparam>
    <rosparam param="target_js_topics">
      ["/curiosity/left_arm/joint_command", "/curiosity/right_arm/joint_command"]
    </rosparam>
  </node>
 *
 * This example shows converting 2 topics to another 2 topics, wherein the name fields are changed respectively.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "roport_msg_converter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  XmlRpc::XmlRpcValue source_joint_names;
  pnh.getParam("source_joint_names", source_joint_names);
  ROS_ASSERT(source_joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_joint_names;
  pnh.getParam("target_joint_names", target_joint_names);
  ROS_ASSERT(target_joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue source_js_topics;
  pnh.getParam("source_js_topics", source_js_topics);
  ROS_ASSERT(source_js_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_js_topics;
  pnh.getParam("target_js_topics", target_js_topics);
  ROS_ASSERT(target_js_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_types;
  pnh.getParam("target_types", target_types);
  if (target_types.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ASSERT(target_js_topics.size() == target_types.size());
    ROS_INFO_NAMED("RoPort Converter", "Custom types defined for the target");
  }

  XmlRpc::XmlRpcValue target_args;
  pnh.getParam("target_args", target_args);
  if (target_args.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ASSERT(target_js_topics.size() == target_args.size());
    ROS_INFO_NAMED("RoPort Converter", "Defined args for publishing to the target topics");
  }

  ROS_ASSERT(source_joint_names.size() > 0 && source_joint_names.size() == target_joint_names.size());
  ROS_ASSERT(source_js_topics.size() > 0 && source_js_topics.size() == target_js_topics.size());
  ROS_ASSERT(source_joint_names.size() == source_js_topics.size());

  // Subscribe source JointState topics and publish them to target topics.
  for (int i = 0; i < source_js_topics.size(); i++) {
    ROS_ASSERT(source_js_topics[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(target_js_topics[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(source_joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(target_joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(source_joint_names[i].size() == target_joint_names[i].size());

    std::vector<std::string> source_names;
    for (size_t j = 0; j < source_joint_names[i].size(); ++j) {
      source_names.push_back(source_joint_names[i][j]);
    }
    std::vector<std::string> target_names;
    for (size_t j = 0; j < target_joint_names[i].size(); ++j) {
      target_names.push_back(target_joint_names[i][j]);
    }

    std::string target_type = "sensor_msgs/JointState";
    if (target_types.size() > 0) {
      target_type = std::string(target_types[i]);
    }
    int target_arg = -1;
    if (target_args.size() > 0) {
      target_arg = int(target_args[i]);
    }

    ros::Publisher publisher;
    if (target_type == "sensor_msgs/JointState") {
      publisher = nh.advertise<sensor_msgs::JointState>(target_js_topics[i], 1);
    } else if (target_type == "franka_core_msgs/JointCommand") {
#ifdef FRANKA_CORE_MSGS
      publisher = nh.advertise<franka_core_msgs::JointCommand>(target_js_topics[i], 1);
#else
      ROS_ERROR_STREAM_NAMED("RoPort Converter", "Request target topic of type: "
          << target_type << ", but the source code is not compiled with this message definition");
      continue;
#endif
    } else {
      ROS_ERROR_STREAM_NAMED("RoPort Converter", "Unknown target topic type: "
          << target_type << " Known are: sensor_msgs/JointState, franka_core_msgs/JointCommand");
      continue;
    }

    ros::Subscriber subscriber = nh.subscribe<sensor_msgs::JointState>(
        source_js_topics[i], 1,
        [publisher, target_type, target_arg, source_names, target_names](auto && PH1) {
          return jointStateCb(
              std::forward<decltype(PH1)>(PH1), publisher, target_type, target_arg, source_names, target_names);}
    );
    publishers_.push_back(publisher);
    subscribers_.push_back(subscriber);
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}

