//
// Created by dongzhipeng on 12/12/21.
//

#include "roport/msg_converter.h"

namespace roport {

MsgConverter::MsgConverter(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
  if (!init()) return;
  starts_.resize(enable_smooth_start_flags_.size());
}

bool MsgConverter::init() {
  XmlRpc::XmlRpcValue source_joint_groups;
  if (!pnh_.getParam("source_joint_groups", source_joint_groups)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'source_joint_groups' is not defined");
    return false;
  }
  ROS_ASSERT(source_joint_groups.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_joint_groups;
  if (!pnh_.getParam("target_joint_groups", target_joint_groups)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'target_joint_groups' is not defined");
    return false;
  }
  ROS_ASSERT(target_joint_groups.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue source_js_topics;
  if (!pnh_.getParam("source_js_topics", source_js_topics)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'source_js_topics' is not defined");
    return false;
  }
  ROS_ASSERT(source_js_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_js_topics;
  if (!pnh_.getParam("target_js_topics", target_js_topics)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'target_js_topics' is not defined");
    return false;
  }
  ROS_ASSERT(target_js_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_types;
  if (!pnh_.getParam("target_types", target_types)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'target_types' is not defined");
    return false;
  }
  ROS_ASSERT(target_types.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue target_args;
  if (!pnh_.getParam("target_args", target_args)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'target_args' is not defined");
    return false;
  }
  ROS_ASSERT(target_args.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue enable_smooth_start;
  if (!pnh_.getParam("enable_smooth_start", enable_smooth_start)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'enable_smooth_start' is not defined");
    return false;
  }
  ROS_ASSERT(enable_smooth_start.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue start_ref_topics;
  if (!pnh_.getParam("start_ref_topics", start_ref_topics)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'start_ref_topics' is not defined");
    return false;
  }
  ROS_ASSERT(start_ref_topics.getType() == XmlRpc::XmlRpcValue::TypeArray);

  XmlRpc::XmlRpcValue start_positions;
  if (!pnh_.getParam("start_positions", start_positions)) {
    ROS_ERROR_STREAM(prefix_ << "Param 'start_positions' is not defined");
    return false;
  }
  ROS_ASSERT(start_positions.getType() == XmlRpc::XmlRpcValue::TypeArray);

  ROS_ASSERT(source_joint_groups.size() > 0);
  ROS_ASSERT(source_joint_groups.size() == target_joint_groups.size() && source_joint_groups.size() == source_js_topics.size() &&
             source_joint_groups.size() == target_js_topics.size() && source_joint_groups.size() == target_types.size() &&
             source_joint_groups.size() == target_args.size() && source_joint_groups.size() == enable_smooth_start.size() &&
             source_joint_groups.size() == start_ref_topics.size() && source_joint_groups.size() == start_positions.size());

  // Subscribe source JointState topics and publish them to target topics.
  // If enable_reflex_ is set, the values to be published will first be smoothed.
  for (int group_id = 0; group_id < source_js_topics.size(); group_id++) {
    ROS_ASSERT(source_js_topics[group_id].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(target_js_topics[group_id].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(source_joint_groups[group_id].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(target_joint_groups[group_id].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(source_joint_groups[group_id].size() == target_joint_groups[group_id].size());

    std::vector<std::string> source_names;
    source_names.reserve(source_joint_groups[group_id].size());
    for (int j = 0; j < source_joint_groups[group_id].size(); ++j) {
      source_names.push_back(source_joint_groups[group_id][j]);
    }
    std::vector<std::string> target_names;
    target_names.reserve(target_joint_groups[group_id].size());
    for (int j = 0; j < target_joint_groups[group_id].size(); ++j) {
      target_names.push_back(target_joint_groups[group_id][j]);
    }

    auto target_type = std::string(target_types[group_id]);
    auto target_arg = int(target_args[group_id]) >= 0 ? int(target_args[group_id]) : -1;
    auto smooth_start_flag = int(enable_smooth_start[group_id]) > 0 ? int(enable_smooth_start[group_id]) : 0;

    std::vector<double> max_vel, max_acc, max_jerk;
    if (smooth_start_flag > 0) {
      if (!phaseJointParameterMap<double>("max_vel", source_names, target_names, max_vel)) {
        return false;
      }
      if (!phaseJointParameterMap<double>("max_acc", source_names, target_names, max_acc)) {
        return false;
      }
      if (!phaseJointParameterMap<double>("max_jerk", source_names, target_names, max_jerk)) {
        return false;
      }
      ROS_ASSERT(!max_vel.empty() && max_vel.size() == max_acc.size() && max_vel.size() == max_jerk.size());
    }

    std::vector<double> start_position;
    start_position.reserve(source_names.size());
    for (int j = 0; j < source_names.size(); ++j) {
      start_position.push_back(start_positions[group_id][j]);
    }
    rotools::RuckigOptimizer* ro;
    if (smooth_start_flag > 0) {
      enable_smooth_start_flags_.push_back(true);
      auto subscriber =
          nh_.subscribe<sensor_msgs::JointState>(start_ref_topics[group_id], 1, [this, group_id, start_position](auto&& PH1) {
            return startCb(std::forward<decltype(PH1)>(PH1), group_id, start_position);
          });
      start_ref_subscribers_.push_back(subscriber);
      smooth_started_flags_.push_back(false);
      ro = new rotools::RuckigOptimizer(static_cast<int>(source_names.size()), max_vel, max_acc, max_jerk);
    } else {
      enable_smooth_start_flags_.push_back(false);
      ros::Subscriber dummy_subscriber;
      start_ref_subscribers_.push_back(dummy_subscriber);
      smooth_started_flags_.push_back(true);
    }
    optimizers_.push_back(ro);

    ros::Publisher publisher;
    if (target_type == msg_type_map_[SENSOR_MSGS_JOINT_STATE]) {
      publisher = nh_.advertise<sensor_msgs::JointState>(target_js_topics[group_id], 1);
    } else if (target_type == msg_type_map_[FRANKA_CORE_MSGS_JOINT_COMMAND]) {
#ifdef FRANKA_CORE_MSGS
      publisher = nh.advertise<franka_core_msgs::JointCommand>(target_js_topics_[group_id], 1);
#else
      ROS_ERROR_STREAM(prefix_ << "Request target topic of type: " << target_type
                               << ", but the source code is not compiled with this message definition");
      continue;
#endif
    } else if (target_type == msg_type_map_[UBT_CORE_MSGS_JOINT_COMMAND]) {
#ifdef UBT_CORE_MSGS
      publisher = nh_.advertise<ubt_core_msgs::JointCommand>(target_js_topics[group_id], 1);
#else
      ROS_ERROR_STREAM(prefix_ << "Request target topic of type: " << target_type
                               << ", but the source code is not compiled with this message definition");
      continue;
#endif
    } else {
      ROS_ERROR_STREAM(prefix_ << "Unknown target topic type: " << target_type);
      continue;
    }

    ros::Subscriber subscriber = nh_.subscribe<sensor_msgs::JointState>(
        source_js_topics[group_id], 1,
        [this, group_id, publisher, target_type, target_arg, source_names, target_names](auto&& PH1) {
          return jointStateCb(std::forward<decltype(PH1)>(PH1), group_id, publisher, target_type, target_arg, source_names,
                              target_names);
        });
    publishers_.push_back(publisher);
    subscribers_.push_back(subscriber);
  }
  return true;
}

void MsgConverter::jointStateCb(const sensor_msgs::JointState::ConstPtr& msg, const size_t& group_id,
                                const ros::Publisher& publisher, const std::string& type, const int& arg,
                                const std::vector<std::string>& source_names, const std::vector<std::string>& target_names) {
  sensor_msgs::JointState filtered_msg;
  if (!filterJointState(msg, filtered_msg, source_names)) return;

  sensor_msgs::JointState smoothed_msg;
  if (enable_smooth_start_flags_[group_id] && !smooth_started_flags_[group_id]) {
    if (!smoothJointState(filtered_msg, optimizers_[group_id], smoothed_msg)) {
      return;
    }
  } else {
    smoothed_msg = filtered_msg;
  }

  if (type == msg_type_map_[SENSOR_MSGS_JOINT_STATE]) {
    publishJointState(smoothed_msg, publisher, source_names, target_names);
  } else if (type == msg_type_map_[FRANKA_CORE_MSGS_JOINT_COMMAND]) {
    publishFrankaJointCommand(smoothed_msg, publisher, arg, source_names, target_names);
  } else if (type == msg_type_map_[UBT_CORE_MSGS_JOINT_COMMAND]) {
    publishUBTJointCommand(smoothed_msg, publisher, arg, source_names, target_names);
  } else {
    ROS_ERROR_STREAM_ONCE(prefix_ << "Unknown target topic type: " << type);
  }
}

bool MsgConverter::filterJointState(const sensor_msgs::JointState::ConstPtr& src_msg, sensor_msgs::JointState& filtered_msg,
                                    const std::vector<std::string>& source_names) {
  filtered_msg.header = src_msg->header;
  filtered_msg.name = source_names;

  if (src_msg->position.empty()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source JointState message defines no position");
    return false;
  }
  if (src_msg->position.size() < source_names.size()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source JointState message have fewer positions (" << src_msg->position.size()
                                         << ") than expected (" << source_names.size() << ")");
    return false;
  }

  size_t i = 0;
  for (auto& name : source_names) {
    auto result = findInVector(src_msg->name, name);
    if (result.first) {
      filtered_msg.position.push_back(src_msg->position[result.second]);
      if (src_msg->velocity.size() == src_msg->position.size()) {
        filtered_msg.velocity.push_back(src_msg->velocity[result.second]);
      }
      if (src_msg->effort.size() == src_msg->position.size()) {
        filtered_msg.effort.push_back(src_msg->effort[result.second]);
      }
    } else {
      ROS_WARN_STREAM_ONCE(prefix_ << "No name in the source joint state message match the given source names (print only once)");
      if (src_msg->position.size() == source_names.size()) {
        filtered_msg.position.push_back(src_msg->position[i]);
        if (src_msg->velocity.size() == source_names.size()) {
          filtered_msg.velocity.push_back(src_msg->velocity[i]);
        }
        if (src_msg->effort.size() == source_names.size()) {
          filtered_msg.effort.push_back(src_msg->effort[i]);
        }
      } else {
        ROS_ERROR_STREAM(prefix_ << "Source joint state message defines no " << name);
        return false;
      }
    }
    i++;
  }
  return true;
}

bool MsgConverter::smoothJointState(const sensor_msgs::JointState& msg, rotools::RuckigOptimizer* oto,
                                    sensor_msgs::JointState& smoothed_msg) {
  if (!*oto->initialized_) return false;
  smoothed_msg.header = msg.header;
  smoothed_msg.name = msg.name;

  if (!oto->set(msg.position, msg.velocity)) return false;
  std::vector<double> q_cmd;
  std::vector<double> dq_cmd;
  oto->update(q_cmd, dq_cmd);
  smoothed_msg.position = q_cmd;
  smoothed_msg.velocity = dq_cmd;
  return true;
}

void MsgConverter::startCb(const sensor_msgs::JointState::ConstPtr& msg, const int& group_id, const std::vector<double>& q_d) {
  if (smooth_started_flags_[group_id]) return;

  if (!*optimizers_[group_id]->initialized_) {
    optimizers_[group_id]->init(*msg, q_d);
    starts_[group_id] = std::chrono::steady_clock::now();
    return;
  }

  if (allClose<double>(msg->position, q_d)) {
    smooth_started_flags_[group_id] = true;
    ROS_INFO("Successfully moved group %d to the start configuration.", group_id);
  } else {
    ROS_WARN_THROTTLE(1, "Smoothly moving group %d to the start configuration ...", group_id);
  }

  if (std::chrono::steady_clock::now() - starts_[group_id] > std::chrono::seconds(20)) {
    smooth_started_flags_[group_id] = true;
    ROS_WARN("Unable to smoothly move group %d to the start configuration in 20 sec. Aborted.", group_id);
  }
}

template <class T>
bool MsgConverter::phaseJointParameterMap(const std::string& param_name, const std::vector<std::string>& source_names,
                                          const std::vector<std::string>& target_names, std::vector<T>& param_out) {
  std::map<std::string, T> param_map;
  if (!pnh_.getParam(param_name, param_map)) {
    ROS_ERROR_STREAM(prefix_ << ("Get param %s failed", param_name.c_str()));
    return false;
  }
  for (size_t n = 0; n < source_names.size(); ++n) {
    if (param_map.find(source_names[n]) != param_map.end()) {
      param_out.push_back(param_map[source_names[n]]);
    } else if (param_map.find(target_names[n]) != param_map.end()) {
      param_out.push_back(param_map[target_names[n]]);
    } else {
      ROS_ERROR_STREAM(prefix_ << ("Unable to find %s param for %s(%s)", param_name.c_str(), source_names[n].c_str(),
                                   target_names[n].c_str()));
      return false;
    }
  }
  return true;
}

void MsgConverter::publishJointState(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub,
                                     const std::vector<std::string>& source_names, const std::vector<std::string>& target_names) {
  sensor_msgs::JointState tgt_msg;
  tgt_msg.header = src_msg.header;

  if (src_msg.name.empty()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given source message to target message.
  // i.e., src_msg.name.size() >= source_names.size() is legal
  if (src_msg.name.size() < source_names.size()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source message has fewer joint names (" << src_msg.name.size() << ") than expected ("
                                         << source_names.size() << ")");
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
      ROS_ERROR_STREAM(prefix_ << "Source joint name " << source_names[i]
                               << "does not match any name in the given JointState message");
    }
  }
  pub.publish(tgt_msg);
}

void MsgConverter::publishFrankaJointCommand(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub, const int& arg,
                                             const std::vector<std::string>& source_names,
                                             const std::vector<std::string>& target_names) {
#ifdef FRANKA_CORE_MSGS
  franka_core_msgs::JointCommand tgt_msg;
  tgt_msg.header = src_msg.header;
  std::set<int> supported_modes{tgt_msg.IMPEDANCE_MODE, tgt_msg.POSITION_MODE, tgt_msg.TORQUE_MODE, tgt_msg.VELOCITY_MODE};
  if (supported_modes.find(arg) == supported_modes.end()) {
    ROS_ERROR_STREAM_ONCE(prefix_ << "Mode " << arg << " is not supported by franka_core_msg");
    return;
  } else {
    tgt_msg.mode = arg;
  }
  if (src_msg.name.empty()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source JointState topic have empty name field");
    return;
  }
  // It is possible to convert only a part of the joint values in the given message.
  if (src_msg.name.size() < source_names.size()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Message name field has fewer names than source names");
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
      ROS_ERROR_STREAM(prefix_ << "Source joint name " << source_names[i]
                               << "does not match any name in the given JointState message");
    }
  }
  pub.publish(tgt_msg);
#endif
}

void MsgConverter::publishUBTJointCommand(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub, const int& arg,
                                          const std::vector<std::string>& source_names,
                                          const std::vector<std::string>& target_names) {
#ifdef UBT_CORE_MSGS
  ubt_core_msgs::JointCommand tgt_msg;
  std::set<int> supported_modes{5, 8};
  if (supported_modes.find(arg) == supported_modes.end()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Mode " << arg << " is not supported by UBT msg");
    return;
  } else {
    tgt_msg.mode = arg;
  }
  if (src_msg.name.empty()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source JointState topic have empty name field");
    return;
  }
  if (src_msg.name.size() < source_names.size()) {
    ROS_ERROR_STREAM_THROTTLE(3, prefix_ << "Source message has fewer joint names (" << src_msg.name.size() << ") than expected ("
                                         << source_names.size() << ")");
    return;
  }
  for (size_t i = 0; i < source_names.size(); ++i) {
    auto result = findInVector<std::string>(src_msg.name, source_names[i]);
    if (result.first) {
      tgt_msg.names.push_back(target_names[i]);
      tgt_msg.command.push_back(src_msg.position[result.second]);
    } else {
      ROS_ERROR_STREAM(prefix_ << "Source joint name " << source_names[i]
                               << "does not match any name in the given JointState message");
    }
  }
  pub.publish(tgt_msg);
#endif
}

}  // namespace roport