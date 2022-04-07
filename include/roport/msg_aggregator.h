//
// Created by Ray on 06/04/2022.
//

#ifndef SRC_MSG_AGGREGATOR_H
#define SRC_MSG_AGGREGATOR_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace roport {
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState,
                                                        sensor_msgs::JointState,
                                                        sensor_msgs::JointState,
                                                        sensor_msgs::JointState,
                                                        sensor_msgs::JointState,
                                                        sensor_msgs::JointState>
    CuriJointStateSyncPolicy;

class MsgAggregator {
 public:
  MsgAggregator(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~MsgAggregator() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  bool have_velocity_head_;
  bool have_velocity_panda_left_;
  bool have_velocity_panda_left_finger_;
  bool have_velocity_panda_right_;
  bool have_velocity_panda_right_finger_;
  bool have_velocity_curi_torso_;
  bool have_effort_head_;
  bool have_effort_panda_left_;
  bool have_effort_panda_left_finger_;
  bool have_effort_panda_right_;
  bool have_effort_panda_right_finger_;
  bool have_effort_curi_torso_;

  std::array<std::string, 7> jointNames_;
  std::string name_prefix_head_;
  std::string name_prefix_panda_left_;
  std::string name_prefix_panda_left_finger_;
  std::string name_prefix_panda_right_;
  std::string name_prefix_panda_right_finger_;
  std::string name_prefix_torso_;

  message_filters::Subscriber<sensor_msgs::JointState>* panda_left_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>* panda_right_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>* curi_torso_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>* panda_left_finger_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>* panda_right_finger_sub_;
  message_filters::Subscriber<sensor_msgs::JointState>* curi_head_sub_;
  message_filters::Synchronizer<CuriJointStateSyncPolicy>* joint_states_sync_;
  ros::Publisher curi_joint_states_pub_;

  void curiJointStatesCB(const sensor_msgs::JointState::ConstPtr& curi_head,
                         const sensor_msgs::JointState::ConstPtr& panda_left,
                         const sensor_msgs::JointState::ConstPtr& panda_left_finger,
                         const sensor_msgs::JointState::ConstPtr& panda_right,
                         const sensor_msgs::JointState::ConstPtr& panda_right_finger,
                         const sensor_msgs::JointState::ConstPtr& curi_torso);
};
}  // namespace roport

#endif  // SRC_MSG_AGGREGATOR_H
