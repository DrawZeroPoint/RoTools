#include "roport/rosout_logger.h"

namespace bt {
std::atomic<bool> RosoutLogger::ref_count(false);

RosoutLogger::RosoutLogger(BT::TreeNode* root_node, ros::console::Level verbosity_level)
    : StatusChangeLogger(root_node), level_(verbosity_level) {
  bool expected = false;
  if (!ref_count.compare_exchange_strong(expected, true)) {
    throw std::logic_error("Only a single instance of RosoutLogger shall be created");
  }
}

auto RosoutLogger::getLevel() const -> ros::console::Level {
  return level_;
}

void RosoutLogger::setLevel(ros::console::Level level) {
  if (level != ros::console::Level::Debug && level != ros::console::Level::Info) {
    throw std::invalid_argument("RosoutLogger::setLevel accepts only Debug or Info");
  }
  level_ = level;
}

RosoutLogger::~RosoutLogger() {
  ref_count.store(false);
}

void RosoutLogger::callback(BT::Duration /*timestamp*/,
                            const BT::TreeNode& node,
                            BT::NodeStatus prev_status,
                            BT::NodeStatus status) {
  constexpr const char* kWhitespaces = "                         ";
  const size_t kWsCount = strlen(kWhitespaces) - 1;

  const auto& node_name = node.name();

  switch (level_) {
    case ros::console::Level::Debug:
      ROS_DEBUG("[%s%s]: %s -> %s", node_name.c_str(), &kWhitespaces[std::min(kWsCount, node_name.size())],
                toStr(prev_status, true).c_str(), toStr(status, true).c_str());
      break;

    case ros::console::Level::Info:
      ROS_INFO("[%s%s]: %s -> %s", node_name.c_str(), &kWhitespaces[std::min(kWsCount, node_name.size())],
               toStr(prev_status, true).c_str(), toStr(status, true).c_str());
      break;
    default:
      ROS_INFO("[%s%s]: %s -> %s", node_name.c_str(), &kWhitespaces[std::min(kWsCount, node_name.size())],
               toStr(prev_status, true).c_str(), toStr(status, true).c_str());
      break;
  }
}

void RosoutLogger::flush() {}

}  // namespace bt
