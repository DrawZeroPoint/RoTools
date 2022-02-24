#ifndef BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

namespace BT {

/** Helper Node to call an actionlib::SimpleActionClient<>
 * inside a BT::ActionNode.
 *
 * Note that the user must implement the methods:
 *
 *  - sendGoal
 *  - onResult
 *  - onFailedRequest
 *  - halt (optionally)
 *
 */
template <class ActionT>
class RosActionNode : public BT::ActionNodeBase {
 protected:
  RosActionNode(ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(name, conf), node_(node_handle), time_sec_(0), time_nsec_(0), name_(name) {
    const std::string kServerName = getInput<std::string>("server_name").value();
    action_client_ = std::make_shared<ActionClientType>(node_, kServerName, true);
    clock_subscriber_ = node_.subscribe("/clock", 1, &RosActionNode::clockCb, this);
  }

 public:
  using BaseClass = RosActionNode<ActionT>;
  using ActionClientType = actionlib::SimpleActionClient<ActionT>;
  using ActionType = ActionT;
  using GoalType = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;

  RosActionNode() = delete;
  ~RosActionNode() override = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static auto providedPorts() -> PortsList {
    const int kPortTimeout = 1000;
    return {InputPort<std::string>("server_name", "name of the Action Server"),
            InputPort<unsigned>("timeout", kPortTimeout, "timeout to connect (milliseconds)")};
  }

  /// Method called when the Action makes a transition from IDLE to RUNNING.
  /// If it return false, the entire action is immediately aborted, it returns
  /// FAILURE and no request is sent to the server.
  virtual auto onSendGoal(GoalType& goal) -> bool = 0;

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual auto onResult(const ResultType& res) -> NodeStatus = 0;

  enum FailureCause { kMissingServer = 0, kAbortedByServer = 1, kRejectedByServer = 2 };

  /// Called when a service call failed. Can be override by the user.
  virtual auto onFailedRequest(FailureCause failure) -> NodeStatus {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return NodeStatus::FAILURE;
  }

  /// If you override this method, you MUST call this implementation invoking:
  ///
  ///    BaseClass::halt()
  ///
  void halt() override {
    if (status() == NodeStatus::RUNNING) {
      action_client_->cancelGoal();
    }
    setStatus(NodeStatus::IDLE);
  }

 private:
  std::shared_ptr<ActionClientType> action_client_;
  ros::NodeHandle& node_;

  auto tick() -> BT::NodeStatus override {
    unsigned msec = getInput<unsigned>("timeout").value();
    const double kMultiplier = 1e-3;
    ros::Duration timeout(static_cast<double>(msec) * kMultiplier);

    bool connected = action_client_->waitForServer(timeout);
    if (!connected) {
      return onFailedRequest(kMissingServer);
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      GoalType goal;
      bool valid_goal = onSendGoal(goal);
      if (!valid_goal) {
        return NodeStatus::FAILURE;
      }
      action_client_->sendGoal(goal);
    }

    // RUNNING
    auto action_state = action_client_->getState();

    // Please refer to these states

    if (action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE) {
      return NodeStatus::RUNNING;
    }
    if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return onResult(*action_client_->getResult());
    }
    if (action_state == actionlib::SimpleClientGoalState::ABORTED) {
      return onFailedRequest(kAbortedByServer);
    }
    if (action_state == actionlib::SimpleClientGoalState::REJECTED) {
      return onFailedRequest(kRejectedByServer);
    }
    // FIXME: is there any other valid state we should consider?
    throw std::logic_error("Unexpected state in RosActionNode::tick()");
  }

  std::string name_;
  uint time_sec_;
  uint time_nsec_;
  ros::Subscriber clock_subscriber_;

  void clockCb(const rosgraph_msgs::ClockConstPtr& msg) {
    time_sec_ = msg->clock.sec;
    time_nsec_ = msg->clock.nsec;
  }
};

/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT>
static void registerRosAction(BT::BehaviorTreeFactory& factory,
                              const std::string& registration_id,
                              ros::NodeHandle& node_handle) {
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_id;
  const auto& basic_ports = RosActionNode<typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());
  factory.registerBuilder(manifest, builder);
}

}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
