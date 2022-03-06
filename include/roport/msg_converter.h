//
// Created by dongzhipeng on 12/12/21.
//

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

  bool getParam(const std::string& param_name, XmlRpc::XmlRpcValue& param_value) {
    if (!pnh_.getParam(param_name, param_value)) {
      if (!nh_.getParam(param_name, param_value)) {
        ROS_ERROR_STREAM(prefix << "Param " << param_name << " is not defined");
        return false;
      }
    }
    return true;
  }

  template <class T>
  bool getParam(const std::string& param_name, std::map<std::string, T>& param_value) {
    if (!pnh_.getParam(param_name, param_value)) {
      if (!nh_.getParam(param_name, param_value)) {
        ROS_ERROR_STREAM(prefix << "Param " << param_name << " is not defined");
        return false;
      }
    }
    return true;
  }

  auto init() -> bool;

  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg,
                    const size_t& group_id,
                    const ros::Publisher& publisher,
                    const std::string& type,
                    const int& arg,
                    const std::vector<std::string>& source_names,
                    const std::vector<std::string>& target_names);

  /**
   * Filter the source joint state message and only preserve the joints whose name are in source_names.
   * If no name is defined in src_msg, will map positions, velocities, and efforts to the filtered message if
   * they have the same size as source_names.
   * @param src_msg Source joint state message.
   * @param filtered_msg Filtered joint state message.
   * @param source_names Names of the joints to be selected.
   * @return True if filtering is succeed, false otherwise.
   */
  auto filterJointState(const sensor_msgs::JointState::ConstPtr& src_msg,
                        sensor_msgs::JointState& filtered_msg,
                        const std::vector<std::string>& source_names) -> bool;

  static auto smoothJointState(const sensor_msgs::JointState& msg,
                               rotools::RuckigOptimizer* oto,
                               sensor_msgs::JointState& smoothed_msg) -> bool;

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
                     const std::vector<std::string>& source_names);

  /**
   *
   * @param src_msg
   * @param pub
   * @param source_names
   * @param target_names
   */
  void publishJointState(const sensor_msgs::JointState& src_msg,
                         const ros::Publisher& pub,
                         const std::vector<std::string>& source_names,
                         const std::vector<std::string>& target_names);

  void publishFrankaJointCommand(const sensor_msgs::JointState& src_msg,
                                 const ros::Publisher& pub,
                                 const int& arg,
                                 const std::vector<std::string>& source_names,
                                 const std::vector<std::string>& target_names);

  void publishUBTJointCommand(const sensor_msgs::JointState& src_msg,
                              const ros::Publisher& pub,
                              const int& arg,
                              const std::vector<std::string>& source_names,
                              const std::vector<std::string>& target_names);

  /**
   * Generic function to find an element in vector and also its position. It returns a pair of bool & int.
   * @tparam T
   * @param vecOfElements
   * @param element
   * @return bool: Represents if element is present in vector or not.
   *         int: Represents the index of element in vector if its found else -1
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

  /**
   * Judge if all corresponding elements in the two given vectors are close to each other under the tolerance.
   * @tparam T
   * @param first One vector.
   * @param second Other vector.
   * @param tol Tolerance.
   * @return True if close.
   */
  template <typename T>
  auto allClose(std::vector<T> first, std::vector<T> second, T tol = 0.01) -> bool {
    for (size_t i = 0; i < first.size(); ++i) {
      if (fabs(first[i] - second[i]) > tol) {
        return false;
      }
    }
    return true;
  }
};
}  // namespace roport

#endif  // SRC_MSG_CONVERTER_H
