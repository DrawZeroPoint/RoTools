#ifndef SRC_ROSOUT_LOGGER_H
#define SRC_ROSOUT_LOGGER_H

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/console.h>

namespace bt {
class RosoutLogger : public BT::StatusChangeLogger {
  static std::atomic<bool> ref_count;

 public:
  explicit RosoutLogger(BT::TreeNode* root_node, ros::console::Level verbosity_level = ros::console::Level::Info);

  [[nodiscard]] auto getLevel() const -> ros::console::Level;

  // Accepts only Info and Debug
  void setLevel(ros::console::Level level);

  ~RosoutLogger() override;

  void callback(BT::Duration timestamp,
                const BT::TreeNode& node,
                BT::NodeStatus prev_status,
                BT::NodeStatus status) override;

  void flush() override;

 private:
  ros::console::Level level_;
};

}  // namespace bt

#endif  // SRC_ROSOUT_LOGGER_H
