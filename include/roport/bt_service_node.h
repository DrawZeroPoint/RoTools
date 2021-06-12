#ifndef BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ros/ros.h>
#include <ros/service_client.h>
#include <rosgraph_msgs/Clock.h>


namespace BT
{

/**
 * Base Action to implement a ROS Service
 */
  template<class ServiceT>
  class RosServiceNode : public BT::SyncActionNode
  {
  protected:

    RosServiceNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
      BT::SyncActionNode(name, conf), node_(nh)
    {
      clock_suber_ = node_.subscribe("/clock", 1, &RosServiceNode::clockCb, this);
    }

  public:
    using BaseClass    = RosServiceNode<ServiceT>;
    using ServiceType  = ServiceT;
    using RequestType  = typename ServiceT::Request;
    using ResponseType = typename ServiceT::Response;

    RosServiceNode() = delete;
    virtual ~RosServiceNode() = default;

    // These ports will be added automatically if this Node is
    // registered using RegisterRosAction<DeriveClass>()
    // The ports without a default value (like the service_name port here)
    // must be given a value during implementation
    static PortsList providedPorts() {
      return {
        InputPort<std::string>("service_name", "name of the ROS service"),
        InputPort<unsigned>("timeout", 10000, "timeout to connect to server (milliseconds)")
      };
    }

    // User must implement this method.
    virtual void onSendRequest(RequestType& request) = 0;

    /// Method (to be implemented by the user) to receive the reply.
    /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
    virtual NodeStatus onResponse(const ResponseType& rep) = 0;

    enum FailureCause{
      MISSING_SERVER = 0,
      FAILED_CALL = 1
    };

    /// Called when a service call failed. Can be override by the user.
    virtual NodeStatus onFailedRequest(FailureCause failure) {
      return NodeStatus::FAILURE;
    }

  protected:
    ros::ServiceClient service_client_;
    typename ServiceT::Response reply_;

    // The node that will be used for any ROS operations
    ros::NodeHandle& node_;

    BT::NodeStatus tick() override {
      if(!service_client_.isValid()){
        std::string server = getInput<std::string>("service_name").value();
        service_client_ = node_.serviceClient<ServiceT>(server);
      }

      unsigned msec;
      getInput("timeout", msec);
      ros::Duration timeout(static_cast<double>(msec) * 1e-3);

      bool connected = service_client_.waitForExistence(timeout);
      if(!connected) {
        return onFailedRequest(MISSING_SERVER);
      }

      typename ServiceT::Request request;
      onSendRequest(request);
      bool received = service_client_.call(request, reply_);
      if(!received) {
        return onFailedRequest(FAILED_CALL);
      }
      return onResponse(reply_);
    }

    int time_sec_;
    ros::Subscriber clock_suber_;

  private:
    void clockCb(const rosgraph_msgs::ClockConstPtr &msg) {
      time_sec_ = msg->clock.sec;
    }
  };

  // Method to register the service into a factory.
  // It gives you the opportunity to set the ros::NodeHandle.
  template <class DerivedT> static
  void RegisterRosService(BT::BehaviorTreeFactory& factory,
                          const std::string& registration_ID,
                          ros::NodeHandle& node_handle)
  {
    NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
      return std::make_unique<DerivedT>(node_handle, name, config );
    };

    TreeNodeManifest manifest;
    manifest.type = getType<DerivedT>();
    manifest.ports = DerivedT::providedPorts();
    manifest.registration_ID = registration_ID;
    const auto& basic_ports = RosServiceNode< typename DerivedT::ServiceType>::providedPorts();
    manifest.ports.insert(basic_ports.begin(), basic_ports.end());

    factory.registerBuilder(manifest, builder);
  }


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
