#include <roport/bt_service_node.h>
#include <roport/bt_action_node.h>
#include <roport/bt_generic_types.h>
#include <roport/rosout_logger.h>

// ROS
#include <ros/ros.h>

// Services (RoPort)
#include <roport/ExecuteAddCollisionBox.h>
#include <roport/ExecuteAddCollisionPlane.h>
#include <roport/ExecuteAllPlans.h>
#include <roport/ExecuteAllPoses.h>
#include <roport/ExecuteAttachCollisionBox.h>
#include <roport/ExecuteBinaryAction.h>
#include <roport/ExecuteDetachCollision.h>
#include <roport/ExecuteGroupJointStates.h>
#include <roport/ExecuteGroupNamedStates.h>
#include <roport/ExecuteGroupPlan.h>
#include <roport/ExecuteGroupPose.h>
#include <roport/ExecuteGroupPosition.h>
#include <roport/ExecuteGroupShift.h>
#include <roport/ExecuteMirroredPose.h>
#include <roport/ExecuteRemoveCollision.h>
#include <roport/GetPreparePose.h>
#include <roport/GetTransformedPose.h>
#include <roport/SenseManipulationPoses.h>
#include <roport/VisualizePose.h>

// Actions (customized)


using namespace BT;

class ExecuteAddCollisionBox : public RosServiceNode<roport::ExecuteAddCollisionBox>
{
public:
  ExecuteAddCollisionBox(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAddCollisionBox>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<std::string>("box_name"),
      InputPort<Pose>("box_pose"),
      InputPort<Point>("box_size"),
      InputPort<int>("is_absolute"),
      InputPort<int>("auto_suffix"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("box_name", request.box_name);
    Pose box_pose{};
    getInput<Pose>("box_pose", box_pose);
    request.box_pose = box_pose.toROS();
    Point box_size{};
    getInput<Point>("box_size", box_size);
    request.box_size = box_size.toROS();
    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);
    int auto_suffix;
    getInput<int>("auto_suffix", auto_suffix);
    request.auto_suffix = bool(auto_suffix);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAddCollisionPlane : public RosServiceNode<roport::ExecuteAddCollisionPlane>
{
public:
  ExecuteAddCollisionPlane(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAddCollisionPlane>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<std::string>("plane_name"),
      InputPort<Pose>("plane_pose"),
      InputPort<Point>("plane_normal"),
      InputPort<int>("auto_suffix"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("plane_name", request.plane_name);
    Pose plane_pose{};
    getInput<Pose>("plane_pose", plane_pose);
    request.plane_pose = plane_pose.toROS();
    Point plane_normal{};
    getInput<Point>("plane_normal", plane_normal);
    request.plane_normal = plane_normal.toROS();
    int auto_suffix;
    getInput<int>("auto_suffix", auto_suffix);
    request.auto_suffix = bool(auto_suffix);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAllPlans : public RosServiceNode<roport::ExecuteAllPlans>
{
public:
  ExecuteAllPlans(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAllPlans>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<StringArray>("group_names"),
      InputPort<int>("is_absolute"),
      InputPort<PoseArrayArray>("all_poses"),
      InputPort<DoubleArray>("stamps"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    StringArray group_names{};
    getInput<StringArray>("group_names", group_names);
    request.group_names = group_names.toROS();

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    PoseArrayArray all_poses{};
    getInput<PoseArrayArray>("all_poses", all_poses);
    request.all_poses = all_poses.toROS();

    DoubleArray stamps{};
    getInput<DoubleArray>("stamps", stamps);
    request.stamps = stamps.plainToROS();

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAllPoses : public RosServiceNode<roport::ExecuteAllPoses>
{
public:
  ExecuteAllPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAllPoses>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<StringArray>("group_names"),
      InputPort<int>("goal_type"),
      InputPort<PoseArray>("goals"),
      InputPort<DoubleArray>("stamps"),
      InputPort<int>("is_cartesian"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    StringArray group_names{};
    getInput<StringArray>("group_names", group_names);
    request.group_names = group_names.toROS();

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    PoseArray goals{};
    getInput<PoseArray>("goals", goals);
    request.goals = goals.toROS();

    DoubleArray stamps{};
    getInput<DoubleArray>("stamps", stamps);
    request.stamps = stamps.plainToROS();

    int is_cartesian;
    getInput<int>("is_cartesian", is_cartesian);
    request.is_cartesian = bool(is_cartesian);

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteMirroredPose : public RosServiceNode<roport::ExecuteMirroredPose>
{
public:
  ExecuteMirroredPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteMirroredPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("reference_group"),
      InputPort<std::string>("mirror_group"),
      InputPort<int>("goal_type"),
      InputPort<Pose>("goal"),
      InputPort<double>("stamp"),
      InputPort<DoubleArray>("mirror_vector"),
      InputPort<int>("is_cartesian"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("reference_group", request.reference_group);
    getInput<std::string>("mirror_group", request.mirror_group);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    Pose goal{};
    getInput<Pose>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("stamp", request.stamp);

    DoubleArray mirror_vector{};
    getInput<DoubleArray>("mirror_vector", mirror_vector);
    request.mirror_vector = mirror_vector.plainToROS();

    int is_cartesian;
    getInput<int>("is_cartesian", is_cartesian);
    request.is_cartesian = bool(is_cartesian);

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteAttachCollisionBox : public RosServiceNode<roport::ExecuteAttachCollisionBox>
{
public:
  ExecuteAttachCollisionBox(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteAttachCollisionBox>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<std::string>("eef_group_name"),
      InputPort<std::string>("box_name"),
      InputPort<Pose>("box_pose"),
      InputPort<Point>("box_size"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("eef_group_name", request.eef_group_name);
    getInput<std::string>("box_name", request.box_name);
    Pose box_pose{};
    getInput<Pose>("box_pose", box_pose);
    request.box_pose = box_pose.toROS();
    Point box_size{};
    getInput<Point>("box_size", box_size);
    request.box_size = box_size.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteBinaryAction : public RosServiceNode<roport::ExecuteBinaryAction>
{
public:
  ExecuteBinaryAction(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteBinaryAction>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<int>("device_id"),
      InputPort<int>("enable"),
      InputPort<double>("value"),
    };
  }

  void onSendRequest(RequestType &request) override {
    int device_id;
    getInput<int>("device_id", device_id);
    request.device_id = static_cast<uint8_t>(device_id);
    int enable;
    getInput<int>("enable", enable);
    request.enable = bool(enable);
    getInput<double>("value", request.value);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteDetachCollision : public RosServiceNode<roport::ExecuteDetachCollision>
{
public:
  ExecuteDetachCollision(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteDetachCollision>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<std::string>("obj_name"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("obj_name", request.obj_name);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupAngularJointStates : public RosServiceNode<roport::ExecuteGroupJointStates>
{
public:
  ExecuteGroupAngularJointStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupJointStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<DoubleArray>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.degToROS();
    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupLinearJointStates : public RosServiceNode<roport::ExecuteGroupJointStates>
{
public:
  ExecuteGroupLinearJointStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupJointStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<DoubleArray>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.plainToROS();
    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupNamedStates : public RosServiceNode<roport::ExecuteGroupNamedStates>
{
public:
  ExecuteGroupNamedStates(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupNamedStates>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<std::string>("state_name"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("state_name", request.state_name);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupPlan : public RosServiceNode<roport::ExecuteGroupPlan>
{
public:
  ExecuteGroupPlan(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupPlan>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<int>("goal_type"),
      InputPort<PoseArray>("poses"),
      InputPort<double>("stamp"),
      InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = static_cast<uint8_t>(goal_type);

    PoseArray poses{};
    getInput<PoseArray>("poses", poses);
    request.poses = poses.toROS();

    getInput<double>("stamp", request.stamp);

    int allow_collision;
    getInput<int>("allow_collision", allow_collision);
    request.allow_collision = bool(allow_collision);

    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupPose : public RosServiceNode<roport::ExecuteGroupPose>
{
public:
  ExecuteGroupPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<int>("goal_type"),
      InputPort<Pose>("goal"),
      InputPort<double>("tolerance"),
      InputPort<std::string>("constraint"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    Pose goal{};
    getInput<Pose>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("tolerance", request.tolerance);
    getInput<std::string>("constraint", request.constraint);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupPosition : public RosServiceNode<roport::ExecuteGroupPosition>
{
public:
  ExecuteGroupPosition(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupPosition>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<int>("goal_type"),
      InputPort<Point>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    Point goal{};
    getInput<Point>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteGroupShift : public RosServiceNode<roport::ExecuteGroupShift>
{
public:
  ExecuteGroupShift(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteGroupShift>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("group_name"),
      InputPort<int>("is_absolute"),
      InputPort<std::string>("axis"),
      InputPort<double>("goal"),
      InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("group_name", request.group_name);

    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    getInput<std::string>("axis", request.axis);
    getInput<double>("goal", request.goal);

    getInput<double>("tolerance", request.tolerance);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class ExecuteRemoveCollision : public RosServiceNode<roport::ExecuteRemoveCollision>
{
public:
  ExecuteRemoveCollision(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::ExecuteRemoveCollision>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("obj_name"),
      InputPort<int>("is_exact"),
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("obj_name", request.obj_name);
    int is_exact;
    getInput<int>("is_exact", is_exact);
    request.is_exact = bool(is_exact);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class GetPreparePose : public RosServiceNode<roport::GetPreparePose>
{
public:
  GetPreparePose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::GetPreparePose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<Pose>("pose"),
      InputPort<int>("is_absolute"),
      InputPort<Point>("shift"),
      OutputPort<Pose>("pre_pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    int is_absolute;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = static_cast<bool>(is_absolute);
    Point shift{};
    getInput<Point>("shift", shift);
    request.shift = shift.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose pre_pose{};
      pre_pose.fromROS(response.pre_pose);
      setOutput("pre_pose", pre_pose);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class GetTransformedPose : public RosServiceNode<roport::GetTransformedPose>
{
public:
  GetTransformedPose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::GetTransformedPose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<Pose>("pose"),
      InputPort<std::string>("source_frame"),
      InputPort<std::string>("target_frame"),
      OutputPort<Pose>("trans_pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    getInput<std::string>("source_frame", request.source_frame);
    getInput<std::string>("target_frame", request.target_frame);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      Pose trans_pose{};
      trans_pose.fromROS(response.trans_pose);
      setOutput("trans_pose", trans_pose);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class SenseManipulationPoses : public RosServiceNode<roport::SenseManipulationPoses>
{
public:
  SenseManipulationPoses(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::SenseManipulationPoses>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<StringArray>("device_names"),
      InputPort<int>("algorithm_id"),
      OutputPort<PoseArray>("poses"),
      OutputPort<Pose>("best_pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    StringArray device_names{};
    getInput<StringArray>("device_names", device_names);
    request.device_names = device_names.toROS();
    int algorithm_id;
    getInput<int>("algorithm_id", algorithm_id);
    request.algorithm_id = static_cast<uint8_t>(algorithm_id);
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      PoseArray poses{};
      poses.fromROS(response.poses);
      setOutput("poses", poses);
      Pose best_pose{};
      best_pose.fromROS(response.best_pose);
      setOutput("best_pose", best_pose);
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

class VisualizePose : public RosServiceNode<roport::VisualizePose>
{
public:
  VisualizePose(ros::NodeHandle &nh, const std::string& name, const BT::NodeConfiguration & cfg) :
    RosServiceNode<roport::VisualizePose>(nh, name, cfg), name_(name) {}

  static BT::PortsList providedPorts() {
    return {
      InputPort<Header>("header"),
      InputPort<std::string>("frame"),
      InputPort<PoseArray>("poses"),
      InputPort<Pose>("pose")
    };
  }

  void onSendRequest(RequestType &request) override {
    getInput<std::string>("frame", request.frame);
    PoseArray poses{};
    getInput<PoseArray>("poses", poses);
    request.poses = poses.toROS();
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    ROS_INFO("RoPort: %s sending request.", name_.c_str());
  }

  BT::NodeStatus onResponse(const ResponseType &response) override {
    if (response.result_status == response.SUCCEEDED) {
      ROS_INFO("RoPort: %s response SUCCEEDED.", name_.c_str());
      return NodeStatus::SUCCESS;
    } else {
      ROS_INFO("RoPort: %s response FAILURE.", name_.c_str());
      return NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
    ROS_ERROR("RoPort: %s request failed %d.", name_.c_str(), static_cast<int>(failure));
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roport_bt_port");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.getParam("tree_file", tree_file);
  if (tree_file.empty()) {
    ROS_ERROR("RoPort: No valid tree file.");
    return -1;
  }

  BT::BehaviorTreeFactory factory;
  RegisterRosService<ExecuteAddCollisionBox>(factory, "ExecuteAddCollisionBox", nh);
  RegisterRosService<ExecuteAddCollisionPlane>(factory, "ExecuteAddCollisionPlane", nh);
  RegisterRosService<ExecuteAllPlans>(factory, "ExecuteAllPlans", nh);
  RegisterRosService<ExecuteAllPoses>(factory, "ExecuteAllPoses", nh);
  RegisterRosService<ExecuteMirroredPose>(factory, "ExecuteMirroredPose", nh);
  RegisterRosService<ExecuteAttachCollisionBox>(factory, "ExecuteAttachCollisionBox", nh);
  RegisterRosService<ExecuteBinaryAction>(factory, "ExecuteBinaryAction", nh);
  RegisterRosService<ExecuteDetachCollision>(factory, "ExecuteDetachCollision", nh);
  RegisterRosService<ExecuteGroupAngularJointStates>(factory, "ExecuteGroupAngularJointStates", nh);
  RegisterRosService<ExecuteGroupLinearJointStates>(factory, "ExecuteGroupLinearJointStates", nh);
  RegisterRosService<ExecuteGroupNamedStates>(factory, "ExecuteGroupNamedStates", nh);
  RegisterRosService<ExecuteGroupPlan>(factory, "ExecuteGroupPlan", nh);
  RegisterRosService<ExecuteGroupPose>(factory, "ExecuteGroupPose", nh);
  RegisterRosService<ExecuteGroupPosition>(factory, "ExecuteGroupPosition", nh);
  RegisterRosService<ExecuteGroupShift>(factory, "ExecuteGroupShift", nh);
  RegisterRosService<ExecuteRemoveCollision>(factory, "ExecuteRemoveCollision", nh);
  RegisterRosService<GetPreparePose>(factory, "GetPreparePose", nh);
  RegisterRosService<GetTransformedPose>(factory, "GetTransformedPose", nh);
  RegisterRosService<SenseManipulationPoses>(factory, "SenseManipulationPoses", nh);
  RegisterRosService<VisualizePose>(factory, "VisualizePose", nh);

  auto tree = factory.createTreeFromFile(tree_file);
  BT::PublisherZMQ publisher_zmp(tree);

  RosoutLogger logger(tree.rootNode());
  printTreeRecursively(tree.rootNode());

  ROS_WARN("Get ready and press Enter to run the task.");
  std::cin.get();

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  while(ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}