#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <boost/bind.hpp>
#include <map>

#include "roport/online_trajectory_optimizer.h"

#ifdef FRANKA_CORE_MSGS
#include <franka_core_msgs/JointCommand.h>
#endif

#ifdef UBT_CORE_MSGS
#include <ubt_core_msgs/JointCommand.h>
#endif

#ifdef YOUR_CUSTOM_MSGS
// Include your message headers
#endif

std::vector<ros::Publisher> publishers_;
std::vector<ros::Subscriber> subscribers_;
std::vector<rotools::OnlineTrajectoryOptimizer*> optimizers_;

enum MSG_TYPES {
  SENSOR_MSGS_JOINT_STATE,
  UBT_CORE_MSGS_JOINT_COMMAND,
  FRANKA_CORE_MSGS_JOINT_COMMAND,
};

std::map<MSG_TYPES, std::string> msg_type_map_ = {
    {SENSOR_MSGS_JOINT_STATE, "sensor_msgs/JointState"},
    {UBT_CORE_MSGS_JOINT_COMMAND, "ubt_core_msgs/JointCommand"},
    {FRANKA_CORE_MSGS_JOINT_COMMAND, "franka_core_msgs/JointCommand"},
};

const std::string MODULE = "RoPort Converter: ";

/**
 * Generic function to find an element in vector and also its position. It returns a pair of bool & int.
 * @tparam T
 * @param vecOfElements
 * @param element
 * @return bool: Represents if element is present in vector or not.
 *         int: Represents the index of element in vector if its found else -1
 */
template <typename T>
std::pair<bool, int> findInVector(const std::vector<T>& vecOfElements, const T& element) {
  std::pair<bool, int> result;
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

/**
 *
 * @param src_msg
 * @param pub
 * @param source_names
 * @param target_names
 */
void publishJointState(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub,
                       const std::vector<std::string>& source_names, const std::vector<std::string>& target_names) {
  sensor_msgs::JointState tgt_msg;
  tgt_msg.header = src_msg.header;

  if (src_msg.name.empty()) {
    ROS_ERROR_STREAM_ONCE_NAMED("RoPort Converter", MODULE << "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given source message to target message.
  if (src_msg.name.size() < source_names.size()) {
    ROS_ERROR_STREAM_NAMED("RoPort Converter", MODULE << "Source message has fewer names than expected");
    return;
  }
  for (size_t i = 0; i < source_names.size(); ++i) {
    auto result = findInVector<std::string>(src_msg.name, source_names[i]);
    if (result.first) {
      tgt_msg.name.push_back(target_names[i]);
      tgt_msg.position.push_back(src_msg.position[result.second]);
      if (src_msg.velocity.size() == src_msg.position.size()) {
        tgt_msg.velocity.push_back(src_msg.velocity[result.second]);
      }
      if (src_msg.effort.size() == src_msg.position.size()) {
        tgt_msg.effort.push_back(src_msg.effort[result.second]);
      }
    } else {
      ROS_ERROR_STREAM_NAMED("RoPort Converter", MODULE << "Source joint name " << source_names[i]
                                                         << "does not match any name in the given JointState message");
    }
  }
  pub.publish(tgt_msg);
}

void publishFrankaJointCommand(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub, const int& arg,
                               const std::vector<std::string>& source_names, const std::vector<std::string>& target_names) {
#ifdef FRANKA_CORE_MSGS
  franka_core_msgs::JointCommand tgt_msg;
  tgt_msg.header = src_msg.header;
  std::set<int> supported_modes{tgt_msg.IMPEDANCE_MODE, tgt_msg.POSITION_MODE, tgt_msg.TORQUE_MODE, tgt_msg.VELOCITY_MODE};
  if (supported_modes.find(arg) == supported_modes.end()) {
    ROS_ERROR_STREAM_ONCE_NAMED("RoPort Converter", "Mode " << arg << " is not supported");
    return;
  } else {
    tgt_msg.mode = arg;
  }
  if (src_msg.name.empty()) {
    ROS_ERROR_ONCE_NAMED("RoPort Converter", "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given message.
  if (src_msg.name.size() < source_names.size()) {
    ROS_ERROR_NAMED("RoPort Converter", "Message name field has fewer names than source names");
    return;
  }
  for (size_t i = 0; i < source_names.size(); ++i) {
    auto result = findInVector<std::string>(src_msg.name, source_names[i]);
    if (result.first) {
      tgt_msg.names.push_back(target_names[i]);
      tgt_msg.position.push_back(src_msg.position[result.second]);
      if (src_msg.velocity.size() == src_msg.position.size()) {
        tgt_msg.velocity.push_back(src_msg.velocity[result.second]);
      }
      if (src_msg.effort.size() == src_msg.position.size()) {
        tgt_msg.effort.push_back(src_msg.effort[result.second]);
      }
    } else {
      ROS_ERROR_STREAM_NAMED("RoPort Converter", "Source joint name "
                                                     << source_names[i]
                                                     << "does not match any name in the given JointState message");
    }
  }
  pub.publish(tgt_msg);
#endif
}

void publishUBTJointCommand(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub, const int& arg,
                            const std::vector<std::string>& source_names, const std::vector<std::string>& target_names) {
#ifdef UBT_CORE_MSGS
  ubt_core_msgs::JointCommand tgt_msg;
  std::set<int> supported_modes{5, 8};
  if (supported_modes.find(arg) == supported_modes.end()) {
    ROS_ERROR_STREAM_ONCE_NAMED("RoPort Converter", MODULE << "Mode " << arg << " is not supported");
    return;
  } else {
    tgt_msg.mode = arg;
  }
  if (src_msg.name.empty()) {
    ROS_ERROR_STREAM_ONCE_NAMED("RoPort Converter", MODULE << "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given message.
  if (src_msg.name.size() < source_names.size()) {
    ROS_ERROR_STREAM_NAMED("RoPort Converter", MODULE << "Message name field has fewer names than source names");
    return;
  }
  for (size_t i = 0; i < source_names.size(); ++i) {
    auto result = findInVector<std::string>(src_msg.name, source_names[i]);
    if (result.first) {
      tgt_msg.names.push_back(target_names[i]);
      tgt_msg.command.push_back(src_msg.position[result.second]);
    } else {
      ROS_ERROR_STREAM_NAMED("RoPort Converter", MODULE << "Source joint name " << source_names[i]
                                                         << "does not match any name in the given JointState message");
    }
  }
  pub.publish(tgt_msg);
#endif
}

void smoothJointState(const sensor_msgs::JointState& msg, rotools::OnlineTrajectoryOptimizer* oto,
                      sensor_msgs::JointState& smoothed_msg) {
  smoothed_msg.header = msg.header;
  smoothed_msg.name = msg.name;
  bool smoothed = false;
  try {
    if (oto->enabled_) {
      std::vector<double> q, dq;
      for (size_t j = 0; j < msg.position.size(); j++) {
        q.push_back(msg.position[j]);
        dq.push_back(msg.velocity[j]);
      }
      if (!oto->initialized_) {
        oto->update(q, dq);
      }
      std::vector<double> q_out, dq_out, ddq_out;
      if (oto->output(q, dq, q_out, dq_out, ddq_out)) {
        for (size_t j = 0; j < msg.position.size(); ++j) {
          smoothed_msg.position.push_back(q_out[j]);
          smoothed_msg.velocity.push_back(dq_out[j]);
        }
        smoothed = true;
      }
    }
  } catch (const std::out_of_range& oor) {
    ROS_ERROR_STREAM("RoPort Converter: Out of Range error: " << oor.what());
  }
  if (!smoothed) {
    smoothed_msg.position = msg.position;
    smoothed_msg.velocity = msg.velocity;
    smoothed_msg.effort = msg.effort;
  }
}

bool filterJointState(const sensor_msgs::JointState::ConstPtr& src_msg, sensor_msgs::JointState& tgt_msg,
                      const std::vector<std::string>& source_names) {
  tgt_msg.header = src_msg->header;
  tgt_msg.name = source_names;
  size_t i = 0;
  for (auto& name : source_names) {
    auto result = findInVector(src_msg->name, name);
    if (result.first) {
      if (src_msg->position.empty() || src_msg->position.size() < source_names.size()) {
        ROS_ERROR_STREAM(MODULE << "Source JointState src_msg defines no position");
        return false;
      }
      tgt_msg.position.push_back(src_msg->position[result.second]);
      if (!src_msg->velocity.empty() && src_msg->velocity.size() == src_msg->position.size()) {
        tgt_msg.velocity.push_back(src_msg->velocity[result.second]);
      }
      if (!src_msg->effort.empty() && src_msg->effort.size() == src_msg->position.size()) {
        tgt_msg.effort.push_back(src_msg->effort[result.second]);
      }
    } else {
      if (src_msg->position.size() == source_names.size()) {
        tgt_msg.position.push_back(src_msg->position[i]);
        if (src_msg->velocity.size() == source_names.size()) {
          tgt_msg.velocity.push_back(src_msg->velocity[i]);
        }
        if (src_msg->effort.size() == source_names.size()) {
          tgt_msg.effort.push_back(src_msg->effort[i]);
        }
      } else {
        ROS_ERROR_STREAM("RoPort Converter: Source JointState src_msg defines no " << name);
        return false;
      }
    }
    i++;
  }
  return true;
}

void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg, const size_t& oto_id, const ros::Publisher& publisher,
                  const std::string& type, const int& arg, const std::vector<std::string>& source_names,
                  const std::vector<std::string>& target_names) {
  sensor_msgs::JointState filtered_msg;
  if (!filterJointState(msg, filtered_msg, source_names)) return;
  sensor_msgs::JointState smoothed_msg;
  smoothJointState(filtered_msg, optimizers_[oto_id], smoothed_msg);
  if (type == msg_type_map_[SENSOR_MSGS_JOINT_STATE]) {
    publishJointState(smoothed_msg, publisher, source_names, target_names);
  } else if (type == msg_type_map_[FRANKA_CORE_MSGS_JOINT_COMMAND]) {
    publishFrankaJointCommand(smoothed_msg, publisher, arg, source_names, target_names);
  } else if (type == msg_type_map_[UBT_CORE_MSGS_JOINT_COMMAND]) {
    publishUBTJointCommand(smoothed_msg, publisher, arg, source_names, target_names);
  } else {
    ROS_ERROR_STREAM_ONCE("RoPort Msg Converter: Unknown target topic type: " << type);
  }
}

template <class T>
bool phaseJointParameterMap(ros::NodeHandle nh, const std::string& param_name, const std::vector<std::string>& source_names,
                            const std::vector<std::string>& target_names, std::vector<T>& param_out) {
  std::map<std::string, T> param_map;
  if (!nh.getParam(param_name, param_map)) {
    ROS_ERROR("RoPort Msg Converter: Get %s failed", param_name.c_str());
    return false;
  }
  for (size_t n = 0; n < source_names.size(); ++n) {
    if (param_map.find(source_names[n]) != param_map.end()) {
      param_out.push_back(param_map[source_names[n]]);
    } else if (param_map.find(target_names[n]) != param_map.end()) {
      param_out.push_back(param_map[target_names[n]]);
    } else {
      ROS_ERROR("RoPort Msg Converter: Unable to find %s param for %s(%s)", param_name.c_str(), source_names[n].c_str(),
                target_names[n].c_str());
      return false;
    }
  }
  return true;
}

/**
 * Convert sensor_msgs::JointState from one topic to another, the name field could change during the convention
 * To use this function, in the launch file, we need define the following parameters:
 *   source_joint_names_: list[list] The outer list is for all joint groups, each joint group is composed by several
 *                       joints that should be measured together and be controlled by a control topic.
 *   target_joint_names_: list[list] This field have one-on-one correspondence with source_joint_names_, the given joint
 *                       values will be under the new names defined by target_joint_names_.
 *   source_js_topics_: list[str] Joint state topics that should be converted to the target_js_topics_.
 *   target_js_topics_: list[str] Topics converted from source_js_topics_.
 *   target_types_: list[str] Target topic types. By default, the target topics will have the type JointState,
 *                 other types are also supported but need modifying the code. Here we support sensor_msgs/JointState
 *                 and franka_core_msgs/JointCommand. If given, its size must be equal to target_js_topics_.
 *   target_args_: list[int] Give one argument for each target topic publishing. This argument often
 *                defines the control type. If given, its size must be equal to target_js_topics_.
 *   enable_reflex_: list[int] If the value>0, Reflexxes will be used to smooth the source js before conversion.
 *
 *   If enable_reflex_ is set, the following params need to be set:
 *
 *   max_vel: map[str, double] For each joint_name, define its maximum velocity. The name could either be source name
 *            or target name (the same for max_acc and max_jerk).
 *   max_acc: map[str, double] For each joint_name, define its maximum acceleration.
 *   max_jerk: map[str, double] For each joint_name, define its maximum jerk.
 *   scales: map[str, double] The keys should be vel, acc, and jerk; the values are the scale factor to be multiplied
 *           to max_vel, max_acc, and max_jerk, respectively.
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
  pnh.getParam("source_joint_names_", source_joint_names);
  ROS_ASSERT(source_joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_joint_names;
  pnh.getParam("target_joint_names_", target_joint_names);
  ROS_ASSERT(target_joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue source_js_topics;
  pnh.getParam("source_js_topics_", source_js_topics);
  ROS_ASSERT(source_js_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_js_topics;
  pnh.getParam("target_js_topics_", target_js_topics);
  ROS_ASSERT(target_js_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_types;
  pnh.getParam("target_types_", target_types);
  ROS_ASSERT(target_types.getType() == XmlRpc::XmlRpcValue::TypeArray && target_js_topics.size() == target_types.size());

  XmlRpc::XmlRpcValue target_args;
  pnh.getParam("target_args_", target_args);
  ROS_ASSERT(target_args.getType() == XmlRpc::XmlRpcValue::TypeArray && target_js_topics.size() == target_args.size());

  XmlRpc::XmlRpcValue enable_reflex;
  pnh.getParam("enable_reflex_", enable_reflex);
  ROS_ASSERT(enable_reflex.getType() == XmlRpc::XmlRpcValue::TypeArray && target_js_topics.size() == enable_reflex.size());

  ROS_ASSERT(source_joint_names.size() > 0 && source_joint_names.size() == target_joint_names.size());
  ROS_ASSERT(source_js_topics.size() > 0 && source_js_topics.size() == target_js_topics.size());
  ROS_ASSERT(source_joint_names.size() == source_js_topics.size());

  // Subscribe source JointState topics and publish them to target topics.
  // If enable_reflex_ is set, the values to be published will first be smoothed.
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

    std::string target_type;
    if (target_types.size() > 0) {
      target_type = std::string(target_types[i]);
    }
    int target_arg = -1;
    if (target_args.size() > 0) {
      target_arg = int(target_args[i]);
    }
    int reflex_flag = 0;
    if (enable_reflex.size() > 0) {
      reflex_flag = int(enable_reflex[i]);
    }

    ros::Publisher publisher;
    if (target_type == msg_type_map_[SENSOR_MSGS_JOINT_STATE]) {
      publisher = nh.advertise<sensor_msgs::JointState>(target_js_topics[i], 1);
    } else if (target_type == msg_type_map_[FRANKA_CORE_MSGS_JOINT_COMMAND]) {
#ifdef FRANKA_CORE_MSGS
      publisher = nh.advertise<franka_core_msgs::JointCommand>(target_js_topics_[i], 1);
#else
      ROS_ERROR_STREAM(MODULE << "Request target topic of type: " << target_type
                               << ", but the source code is not compiled with this message definition");
      continue;
#endif
    } else if (target_type == msg_type_map_[UBT_CORE_MSGS_JOINT_COMMAND]) {
#ifdef UBT_CORE_MSGS
      publisher = nh.advertise<ubt_core_msgs::JointCommand>(target_js_topics[i], 1);
#else
      ROS_ERROR_STREAM(MODULE << "Request target topic of type: " << target_type
                               << ", but the source code is not compiled with this message definition");
      continue;
#endif
    } else {
      ROS_ERROR_STREAM(MODULE << "Unknown target topic type: " << target_type);
      continue;
    }

    // Create reflex object for this topic
    auto* oto = new rotools::OnlineTrajectoryOptimizer(target_names.size(), 500);
    if (reflex_flag > 0) {
      std::vector<double> max_vel, max_acc, max_jerk, scales;
      if (!phaseJointParameterMap<double>(pnh, "max_vel", source_names, target_names, max_vel)) {
        ROS_ERROR_STREAM(MODULE << "Get joint max velocity failed");
        return -1;
      }
      if (!phaseJointParameterMap<double>(pnh, "max_acc", source_names, target_names, max_acc)) {
        ROS_ERROR_STREAM(MODULE << "Get joint max acceleration failed");
        return -1;
      }
      if (!phaseJointParameterMap<double>(pnh, "max_jerk", source_names, target_names, max_jerk)) {
        ROS_ERROR_STREAM(MODULE << "Get joint max jerk failed");
        return -1;
      }
      std::vector<std::string> query_names{"vel", "acc", "jerk"};
      if (!phaseJointParameterMap<double>(pnh, "scales", query_names, query_names, scales)) {
        ROS_ERROR("Get vel_scale, acc_scale, and jerk_scale failed");
        return -1;
      }
      ROS_ASSERT(!max_vel.empty() && !max_acc.empty() && !max_jerk.empty() && scales.size() == 3);
      oto->setParameters(max_vel, max_acc, max_jerk, scales[0], scales[1], scales[2]);
    }
    optimizers_.push_back(oto);

    const size_t oto_id = i;
    ros::Subscriber subscriber = nh.subscribe<sensor_msgs::JointState>(
        source_js_topics[i], 1, [oto_id, publisher, target_type, target_arg, source_names, target_names](auto&& PH1) {
          return jointStateCb(std::forward<decltype(PH1)>(PH1), oto_id, publisher, target_type, target_arg, source_names,
                              target_names);
        });
    publishers_.push_back(publisher);
    subscribers_.push_back(subscriber);
  }

  ros::AsyncSpinner spinner(source_js_topics.size());
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
