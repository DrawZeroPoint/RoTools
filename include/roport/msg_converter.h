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

enum MSG_TYPES {
  SENSOR_MSGS_JOINT_STATE,
  UBT_CORE_MSGS_JOINT_COMMAND,
  FRANKA_CORE_MSGS_JOINT_COMMAND,
};

class MsgConverter {
 public:
  MsgConverter(const ros::NodeHandle& nh, const ros::NodeHandle& pn);
  ~MsgConverter() = default;

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

 private:
  const std::string prefix_{"Msg Converter: "};

  std::vector<ros::Publisher> publishers_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<rotools::RuckigOptimizer*> optimizers_;

  std::vector<bool> enable_smooth_start_flags_;
  std::vector<bool> smooth_started_flags_;
  std::vector<ros::Subscriber> start_ref_subscribers_;

  std::map<MSG_TYPES, std::string> msg_type_map_ = {
      {SENSOR_MSGS_JOINT_STATE, "sensor_msgs/JointState"},
      {UBT_CORE_MSGS_JOINT_COMMAND, "ubt_core_msgs/JointCommand"},
      {FRANKA_CORE_MSGS_JOINT_COMMAND, "franka_core_msgs/JointCommand"},
  };

  std::vector<std::chrono::steady_clock::time_point> starts_;

  bool init();

  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg, const size_t& group_id, const ros::Publisher& publisher,
                    const std::string& type, const int& arg, const std::vector<std::string>& source_names,
                    const std::vector<std::string>& target_names);

  /**
   * Filter the source joint state message and only preserve the joints whose name are in source_names.
   * If no name is defined in src_msg, will map positions, velocities, and efforts to the filtered message if
   * their size is the same as source_names.
   * @param src_msg Source joint state message.
   * @param filtered_msg Filtered joint state message.
   * @param source_names Names of the joints to be selected.
   * @return True if filtering is succeed, false otherwise.
   */
  bool filterJointState(const sensor_msgs::JointState::ConstPtr& src_msg, sensor_msgs::JointState& filtered_msg,
                        const std::vector<std::string>& source_names);

  static bool smoothJointState(const sensor_msgs::JointState& msg, rotools::RuckigOptimizer* oto,
                               sensor_msgs::JointState& smoothed_msg);

  template <class T>
  bool phaseJointParameterMap(const std::string& param_name, const std::vector<std::string>& source_names,
                              const std::vector<std::string>& target_names, std::vector<T>& param_out);

  void startCb(const sensor_msgs::JointState::ConstPtr& msg, const int& group_id, const std::vector<double>& q_d);

  /**
   *
   * @param src_msg
   * @param pub
   * @param source_names
   * @param target_names
   */
  void publishJointState(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub,
                         const std::vector<std::string>& source_names, const std::vector<std::string>& target_names);

  void publishFrankaJointCommand(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub, const int& arg,
                                 const std::vector<std::string>& source_names, const std::vector<std::string>& target_names);

  void publishUBTJointCommand(const sensor_msgs::JointState& src_msg, const ros::Publisher& pub, const int& arg,
                              const std::vector<std::string>& source_names, const std::vector<std::string>& target_names);

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
   * Judge if all corresponding elements in the two given vectors are close to each other under the tolerance.
   * @tparam T
   * @param a One vector.
   * @param b Other vector.
   * @param tol Tolerance.
   * @return True if close.
   */
  template <typename T>
  bool allClose(std::vector<T> a, std::vector<T> b, T tol = 0.01) {
    for (size_t i = 0; i < a.size(); ++i) {
      if (fabs(a[i] - b[i]) > tol) {
        return false;
      }
    }
    return true;
  }
};
}  // namespace roport

#endif  // SRC_MSG_CONVERTER_H
