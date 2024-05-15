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
#include <std_msgs/Float64.h>

namespace roport {
class MsgAggregator {
 public:
  MsgAggregator(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~MsgAggregator() = default;

 private:
  typedef message_filters::Subscriber<sensor_msgs::JointState> MsgSubscriber;

  typedef message_filters::sync_policies::
      ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState, sensor_msgs::JointState>
          TriPolicy;
  typedef message_filters::Synchronizer<TriPolicy> TriPolicySynchronizer;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> DuoPolicy;
  typedef message_filters::Synchronizer<DuoPolicy> DuoPolicySynchronizer;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher publisher_;

  size_t high_frequency_num_;
  std::vector<std::string> high_frequency_topics_;
  std::vector<std::vector<std::string>> high_frequency_name_groups_;
  std::vector<std::shared_ptr<MsgSubscriber>> high_frequency_subscribers_;

  std::shared_ptr<DuoPolicySynchronizer> duo_high_frequency_synchronizer_;
  std::shared_ptr<TriPolicySynchronizer> tri_high_frequency_synchronizer_;

  size_t low_frequency_num_;
  std::vector<std::string> low_frequency_topics_;
  std::vector<std::vector<std::string>> low_frequency_name_groups_;
  std::vector<std::shared_ptr<MsgSubscriber>> low_frequency_subscribers_;

  std::shared_ptr<DuoPolicySynchronizer> duo_low_frequency_synchronizer_;
  std::shared_ptr<TriPolicySynchronizer> tri_low_frequency_synchronizer_;
  sensor_msgs::JointState low_frequency_joint_state_;

  std::vector<std::string> joints_to_inspect_;
  std::vector<ros::Publisher> joint_scaled_torque_publishers_;

  std::vector<double> inspected_joint_torque_limits_;

  const std::string prefix{"Msg Aggregator: "};

  void init();

  void highFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg);

  void highFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1, const sensor_msgs::JointState::ConstPtr& msg_2);

  void highFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1,
                       const sensor_msgs::JointState::ConstPtr& msg_2,
                       const sensor_msgs::JointState::ConstPtr& msg_3);

  void lowFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1, const sensor_msgs::JointState::ConstPtr& msg_2);

  void lowFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1,
                      const sensor_msgs::JointState::ConstPtr& msg_2,
                      const sensor_msgs::JointState::ConstPtr& msg_3);

  void publishInspected(const sensor_msgs::JointState& msg);

  void publishCombined(const sensor_msgs::JointState& high_frequency_msg);
};
}  // namespace roport

#endif  // SRC_MSG_AGGREGATOR_H
