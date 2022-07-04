/*
 * msg_converter
 * Copyright (c) 2021-2022, Zhipeng Dong
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SRC_MSG_CONVERTER_H
#define SRC_MSG_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "roport/online_trajectory_optimizer.h"
#include "ruckig/ruckig.hpp"

#ifdef FRANKA_CORE_MSGS
#include <franka_core_msgs/JointCommand.h>
#endif

#ifdef UBT_CORE_MSGS
#include <ubt_core_msgs/JointCommand.h>
#endif

#ifdef YOUR_CUSTOM_MSGS
// Include your message headers
#endif

namespace roport {

enum MsgTypes {
  kSensorMsgsJointState,
  kUbtCoreMsgsJointCommand,
  kFrankaCoreMsgsJointCommand,
};

class MsgConverter {
 public:
  MsgConverter(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~MsgConverter() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  const std::string prefix{"Msg Converter: "};

  std::vector<ros::Publisher> publishers_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<rotools::RuckigOptimizer*> optimizers_;

  std::vector<bool> enable_smooth_start_flags_;
  std::vector<bool> finished_smooth_start_flags_;
  std::vector<ros::Subscriber> start_ref_subscribers_;

  std::map<MsgTypes, std::string> msg_type_map_ = {
      {kSensorMsgsJointState, "sensor_msgs/JointState"},
      {kUbtCoreMsgsJointCommand, "ubt_core_msgs/JointCommand"},
      {kFrankaCoreMsgsJointCommand, "franka_core_msgs/JointCommand"},
  };

  std::vector<std::chrono::steady_clock::time_point> starts_;

  auto getParam(const std::string& param_name, XmlRpc::XmlRpcValue& param_value) -> bool {
    if (!pnh_.getParam(param_name, param_value)) {
      if (!nh_.getParam(param_name, param_value)) {
        ROS_ERROR_STREAM(prefix << "Param " << param_name << " is not defined");
        return false;
      }
    }
    return true;
  }

  template <class T>
  auto getParam(const std::string& param_name, std::map<std::string, T>& param_value) -> bool {
    if (!pnh_.getParam(param_name, param_value)) {
      if (!nh_.getParam(param_name, param_value)) {
        ROS_ERROR_STREAM(prefix << "Param " << param_name << " is not defined");
        return false;
      }
    }
    return true;
  }

  auto init() -> bool;

  /**
   * Callback function for each of the source_js_topics. If the source msg cannot be filtered or smoothed (if enabled),
   * no target msg will be published.
   * @param msg Source joint state msg.
   * @param group_id The id of the joint group.
   * @param source_topic Name of the source topic.
   * @param reference_topic Name of the reference topic used for smooth start.
   * @param publisher Publisher to target topic.
   * @param type Target msg type.
   * @param arg Target msg arg.
   * @param source_names Cared joint names in the source joint state msg.
   * @param target_names Correspondences of source_names in the target joint state msg.
   */
  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg,
                    const size_t& group_id,
                    const std::string& source_topic,
                    const std::string& reference_topic,
                    const ros::Publisher& publisher,
                    const std::string& type,
                    const int& arg,
                    const std::vector<std::string>& source_names,
                    const std::vector<std::string>& target_names);

  /**
   * Filter the source joint state message and only preserve the joints whose name are in source_names.
   * Names' order in filtered_names could be different with that of src_msg.name. The filtered_msg.name will
   * be the same as filtered_names.
   * If no name is defined in src_msg, will map positions, velocities, and efforts to the filtered message if
   * they have the same size as source_names.
   * @param src_msg Source joint state message.
   * @param filtered_names Names of the joints to be selected.
   * @param source_topic Name of the source topic.
   * @retval filtered_msg Filtered joint state message.
   * @return True if filtering is succeed, false otherwise.
   */
  auto filterJointState(const sensor_msgs::JointState::ConstPtr& src_msg,
                        const std::vector<std::string>& filtered_names,
                        const std::string& source_topic,
                        sensor_msgs::JointState& filtered_msg) -> bool;

  /**
   * Smooth the source joint state msg in the sense of making the smoothed msg gently lead the robot goto the
   * desired configuration described by msg using the online trajectory optimizer.
   * @param msg Source joint state msg.
   * @param source_topic Topic name of the source joint state msg.
   * @param reference_topic Topic name of the reference msg indicating the robot's instantaneous configuration.
   * @param oto Online trajectory optimizer.
   * @retval smoothed_msg Smoothed joint state msg.
   * @return True if smoothed msg is retrieved, false otherwise.
   */
  auto smoothJointState(const sensor_msgs::JointState& msg,
                        const std::string& source_topic,
                        const std::string& reference_topic,
                        rotools::RuckigOptimizer* oto,
                        sensor_msgs::JointState& smoothed_msg) -> bool;

  /**
   * Get the map under the given param_name. Each element in the map is a pair <joint_name, value>.
   * Get all values of specified joint names using this map. The joint names are defined by source_names
   * or their analogues target_names.
   * @tparam T Value type.
   * @param param_name Name of the param containing a map.
   * @param source_names Joint names.
   * @param target_names Joint names.
   * @retval param_out Output param values.
   * @return True if got values. False if the param does not exist or cannot find source/target names in the map.
   */
  template <class T>
  auto phaseJointParameterMap(const std::string& param_name,
                              const std::vector<std::string>& source_names,
                              const std::vector<std::string>& target_names,
                              std::vector<T>& param_out) -> bool;

  /**
   * This callback function constantly monitor the current joint states and compare them with the desired position,
   * if they are close enough, set the smooth_started_flags for group of the given id to true.
   * @param msg Measured joint state msg.
   * @param group_id The id of the controlled group.
   * @param source_names Names of the joints to monitor.
   */
  void smoothStartCb(const sensor_msgs::JointState::ConstPtr& msg,
                     const int& group_id,
                     const std::string& source_topic,
                     const std::vector<std::string>& source_names);

  /**
   * Given a sensor_msg::JointState type msg, convert values from source_names to target_names and publish
   * the converted msg using the given publisher.
   * @param src_msg Source msg.
   * @param publisher The publisher.
   * @param source_names Names to be selected from src_msg.name
   * @param target_names Names corresponding to source_names while taking the selected values.
   */
  void publishJointState(const sensor_msgs::JointState& src_msg,
                         const ros::Publisher& publisher,
                         const std::vector<std::string>& source_names,
                         const std::vector<std::string>& target_names);

  void publishFrankaJointCommand(const sensor_msgs::JointState& src_msg,
                                 const ros::Publisher& publisher,
                                 const int& arg,
                                 const std::vector<std::string>& source_names,
                                 const std::vector<std::string>& target_names);

  void publishUBTJointCommand(const sensor_msgs::JointState& src_msg,
                              const ros::Publisher& publisher,
                              const int& arg,
                              const std::vector<std::string>& source_names,
                              const std::vector<std::string>& target_names);

  /**
   * Generic function to find an element in vector and also its position. It returns a pair of bool & int.
   * The function will return if the first match is found.
   * @tparam T Type of the element in the vector.
   * @param vec_of_elements Vector to find the element from.
   * @param element The element to find.
   * @return bool: Represents if element is present in vector or not.
   *         int: Represents the index of element in vector if its found else -1.
   */
  template <typename T>
  auto findInVector(const std::vector<T>& vec_of_elements, const T& element) -> std::pair<bool, int> {
    std::pair<bool, int> result;
    // Find given element in vector
    auto iterator = std::find(vec_of_elements.begin(), vec_of_elements.end(), element);
    if (iterator != vec_of_elements.end()) {
      result.second = distance(vec_of_elements.begin(), iterator);
      result.first = true;
    } else {
      result.first = false;
      result.second = -1;
    }
    return result;
  }
};
}  // namespace roport

#endif  // SRC_MSG_CONVERTER_H
