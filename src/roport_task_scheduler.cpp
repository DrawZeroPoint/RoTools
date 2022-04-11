#include <roport/bt_action_node.h>
#include <roport/bt_generic_types.h>
#include <roport/bt_service_node.h>
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
#include <roport/ExecuteFrankaGripperGrasp.h>
#include <roport/ExecuteGroupJointStates.h>
#include <roport/ExecuteGroupNamedStates.h>
#include <roport/ExecuteGroupPlan.h>
#include <roport/ExecuteGroupPose.h>
#include <roport/ExecuteGroupPosition.h>
#include <roport/ExecuteGroupShift.h>
#include <roport/ExecuteJointPosition.h>
#include <roport/ExecuteManipulationPlanning.h>
#include <roport/ExecuteMirroredPose.h>
#include <roport/ExecutePathPlanning.h>
#include <roport/ExecuteRemoveCollision.h>
#include <roport/GetPreparePose.h>
#include <roport/GetTransformedPose.h>
#include <roport/SenseManipulationPoses.h>
#include <roport/VisualizePose.h>

// Actions (customized)

using namespace BT;

namespace bt {

class ExecuteAddCollisionBox : public RosServiceNode<roport::ExecuteAddCollisionBox> {
 public:
  ExecuteAddCollisionBox(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteAddCollisionBox>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),   InputPort<std::string>("group_name"), InputPort<std::string>("box_name"),
        InputPort<Pose>("box_pose"),   InputPort<Point>("box_size"),         InputPort<int>("is_absolute"),
        InputPort<int>("auto_suffix"),
    };
  }

  void onSendRequest(RequestType& request) override {
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
    request.is_absolute = static_cast<bool>(is_absolute);
    int auto_suffix;
    getInput<int>("auto_suffix", auto_suffix);
    request.auto_suffix = bool(auto_suffix);
  }
};

class ExecuteAddCollisionPlane : public RosServiceNode<roport::ExecuteAddCollisionPlane> {
 public:
  ExecuteAddCollisionPlane(const ros::NodeHandle& node_handle,
                           const std::string& name,
                           const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteAddCollisionPlane>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),   InputPort<std::string>("group_name"), InputPort<std::string>("plane_name"),
        InputPort<Pose>("plane_pose"), InputPort<Point>("plane_normal"),     InputPort<int>("auto_suffix"),
    };
  }

  void onSendRequest(RequestType& request) override {
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
  }
};

class ExecuteAllPlans : public RosServiceNode<roport::ExecuteAllPlans> {
 public:
  ExecuteAllPlans(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteAllPlans>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),      InputPort<StringArray>("group_names"),
        InputPort<int>("is_absolute"),    InputPort<PoseArrayArray>("all_poses"),
        InputPort<DoubleArray>("stamps"), InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType& request) override {
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
  }
};

class ExecuteAllPoses : public RosServiceNode<roport::ExecuteAllPoses> {
 public:
  ExecuteAllPoses(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteAllPoses>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),       InputPort<StringArray>("group_names"), InputPort<int>("goal_type"),
        InputPort<PoseArray>("goals"),     InputPort<DoubleArray>("stamps"),      InputPort<int>("is_cartesian"),
        InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType& request) override {
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
  }
};

class ExecuteMirroredPose : public RosServiceNode<roport::ExecuteMirroredPose> {
 public:
  ExecuteMirroredPose(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteMirroredPose>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
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

  void onSendRequest(RequestType& request) override {
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
  }
};

class ExecuteAttachCollisionBox : public RosServiceNode<roport::ExecuteAttachCollisionBox> {
 public:
  ExecuteAttachCollisionBox(const ros::NodeHandle& node_handle,
                            const std::string& name,
                            const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteAttachCollisionBox>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("eef_group_name"),
        InputPort<std::string>("box_name"),
        InputPort<Pose>("box_pose"),
        InputPort<Point>("box_size"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("eef_group_name", request.eef_group_name);
    getInput<std::string>("box_name", request.box_name);
    Pose box_pose{};
    getInput<Pose>("box_pose", box_pose);
    request.box_pose = box_pose.toROS();
    Point box_size{};
    getInput<Point>("box_size", box_size);
    request.box_size = box_size.toROS();
  }
};

class ExecuteBinaryAction : public RosServiceNode<roport::ExecuteBinaryAction> {
 public:
  ExecuteBinaryAction(ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteBinaryAction>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<int>("device_id"),
        InputPort<int>("enable"),
        InputPort<double>("value"),
    };
  }

  void onSendRequest(RequestType& request) override {
    int device_id;
    getInput<int>("device_id", device_id);
    request.device_id = static_cast<uint8_t>(device_id);
    int enable;
    getInput<int>("enable", enable);
    request.enable = bool(enable);
    getInput<double>("value", request.value);
  }
};

class ExecuteDetachCollision : public RosServiceNode<roport::ExecuteDetachCollision> {
 public:
  ExecuteDetachCollision(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteDetachCollision>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("obj_name"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("obj_name", request.obj_name);
  }
};

class ExecuteFrankaGripperGrasp : public RosServiceNode<roport::ExecuteFrankaGripperGrasp> {
 public:
  ExecuteFrankaGripperGrasp(const ros::NodeHandle& node_handle,
                            const std::string& name,
                            const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteFrankaGripperGrasp>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<double>("width"), InputPort<double>("epsilon_inner"), InputPort<double>("epsilon_outer"),
        InputPort<double>("speed"), InputPort<double>("force"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<double>("width", request.width);
    getInput<double>("epsilon_inner", request.epsilon_inner);
    getInput<double>("epsilon_outer", request.epsilon_outer);
    getInput<double>("speed", request.speed);
    getInput<double>("force", request.force);
  }
};

class ExecuteGroupAngularJointStates : public RosServiceNode<roport::ExecuteGroupJointStates> {
 public:
  ExecuteGroupAngularJointStates(const ros::NodeHandle& node_handle,
                                 const std::string& name,
                                 const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupJointStates>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<DoubleArray>("goal"),
        InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.degToROS();
    getInput<double>("tolerance", request.tolerance);
  }
};

class ExecuteGroupLinearJointStates : public RosServiceNode<roport::ExecuteGroupJointStates> {
 public:
  ExecuteGroupLinearJointStates(const ros::NodeHandle& node_handle,
                                const std::string& name,
                                const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupJointStates>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<DoubleArray>("goal"),
        InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);
    DoubleArray goal{};
    getInput<DoubleArray>("goal", goal);
    request.goal = goal.plainToROS();
    getInput<double>("tolerance", request.tolerance);
  }
};

class ExecuteGroupNamedStates : public RosServiceNode<roport::ExecuteGroupNamedStates> {
 public:
  ExecuteGroupNamedStates(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupNamedStates>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("group_name"),
        InputPort<std::string>("state_name"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);
    getInput<std::string>("state_name", request.state_name);
  }
};

class ExecuteGroupPlan : public RosServiceNode<roport::ExecuteGroupPlan> {
 public:
  ExecuteGroupPlan(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupPlan>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),   InputPort<std::string>("group_name"), InputPort<int>("goal_type"),
        InputPort<PoseArray>("poses"), InputPort<double>("stamp"),           InputPort<int>("allow_collision"),
    };
  }

  void onSendRequest(RequestType& request) override {
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
  }
};

class ExecuteGroupPose : public RosServiceNode<roport::ExecuteGroupPose> {
 public:
  ExecuteGroupPose(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupPose>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"), InputPort<std::string>("group_name"), InputPort<int>("goal_type"),
        InputPort<Pose>("goal"),     InputPort<double>("tolerance"),       InputPort<std::string>("constraint"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    Pose goal{};
    getInput<Pose>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("tolerance", request.tolerance);
    getInput<std::string>("constraint", request.constraint);
  }
};

class ExecutePathPlanning : public RosServiceNode<roport::ExecutePathPlanning> {
 public:
  ExecutePathPlanning(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecutePathPlanning>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {InputPort<Header>("header"),
            InputPort<int>("base_goal_type"),
            InputPort<JointState>("joint_goal_state"),
            InputPort<Pose>("base_goal_pose"),
            InputPort<double>("base_pos_tolerance"),
            InputPort<double>("base_ori_tolerance")};
  }

  void onSendRequest(RequestType& request) override {
    int base_goal_type;
    getInput<int>("base_goal_type", base_goal_type);
    request.base_goal_type = base_goal_type;

    Pose base_goal_pose{};
    getInput<Pose>("base_goal_pose", base_goal_pose);
    request.base_goal_pose = base_goal_pose.toROS();

    JointState joint_goal_state{};
    getInput<JointState>("joint_goal_state", joint_goal_state);
    request.joint_goal_state = joint_goal_state.toROS();

    getInput<double>("base_pos_tolerance", request.base_pos_tolerance);
    getInput<double>("base_ori_tolerance", request.base_ori_tolerance);
  }
};

class ExecuteManipulationPlanning : public RosServiceNode<roport::ExecuteManipulationPlanning> {
 public:
  ExecuteManipulationPlanning(const ros::NodeHandle& node_handle,
                              const std::string& name,
                              const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteManipulationPlanning>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {InputPort<int>("object_goal_pose_type"),   InputPort<int>("base_goal_pose_type"),
            InputPort<JointState>("joint_goal_state"), InputPort<Pose>("object_goal_pose"),
            InputPort<Pose>("base_goal_pose"),         InputPort<double>("base_pos_tolerance"),
            InputPort<double>("base_ori_tolerance"),   InputPort<double>("object_pos_tolerance"),
            InputPort<double>("object_ori_tolerance")};
  }

  void onSendRequest(RequestType& request) override {
    int object_goal_pose_type = 0;
    getInput<int>("object_goal_pose_type", object_goal_pose_type);
    request.object_goal_pose_type = object_goal_pose_type;

    int base_goal_pose_type = 0;
    getInput<int>("base_goal_pose_type", base_goal_pose_type);
    request.base_goal_pose_type = base_goal_pose_type;

    Pose object_goal_pose{};
    getInput<Pose>("object_goal_pose", object_goal_pose);
    request.object_goal_pose = object_goal_pose.toROS();

    Pose base_goal_pose{};
    getInput<Pose>("base_goal_pose", base_goal_pose);
    request.base_goal_pose = base_goal_pose.toROS();

    JointState joint_goal_state{};
    getInput<JointState>("joint_goal_state", joint_goal_state);
    request.joint_goal_state = joint_goal_state.toROS();

    getInput<double>("base_pos_tolerance", request.base_pos_tolerance);
    getInput<double>("base_ori_tolerance", request.base_ori_tolerance);
    getInput<double>("object_pos_tolerance", request.object_pos_tolerance);
    getInput<double>("object_ori_tolerance", request.object_ori_tolerance);
  }
};

class ExecuteGroupPosition : public RosServiceNode<roport::ExecuteGroupPosition> {
 public:
  ExecuteGroupPosition(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupPosition>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"), InputPort<std::string>("group_name"), InputPort<int>("goal_type"),
        InputPort<Point>("goal"),    InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);

    int goal_type;
    getInput<int>("goal_type", goal_type);
    request.goal_type = goal_type;

    Point goal{};
    getInput<Point>("goal", goal);
    request.goal = goal.toROS();

    getInput<double>("tolerance", request.tolerance);
  }
};

class ExecuteGroupShift : public RosServiceNode<roport::ExecuteGroupShift> {
 public:
  ExecuteGroupShift(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteGroupShift>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),   InputPort<std::string>("group_name"),
        InputPort<int>("is_absolute"), InputPort<std::string>("axis"),
        InputPort<double>("goal"),     InputPort<double>("tolerance"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("group_name", request.group_name);

    int is_absolute = 0;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = bool(is_absolute);

    getInput<std::string>("axis", request.axis);
    getInput<double>("goal", request.goal);

    getInput<double>("tolerance", request.tolerance);
  }
};

class ExecuteJointPosition : public RosServiceNode<roport::ExecuteJointPosition> {
 public:
  ExecuteJointPosition(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteJointPosition>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<JointState>("goal_state"),
        InputPort<double>("speed_ratio"),
    };
  }

  void onSendRequest(RequestType& request) override {
    double speed_ratio = 0.1;
    getInput<double>("speed_ratio", speed_ratio);
    request.speed_ratio = speed_ratio;

    JointState goal_state{};
    getInput<JointState>("goal_state", goal_state);
    request.goal_state = goal_state.toROS();
  }
};

class ExecuteRemoveCollision : public RosServiceNode<roport::ExecuteRemoveCollision> {
 public:
  ExecuteRemoveCollision(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::ExecuteRemoveCollision>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {
        InputPort<Header>("header"),
        InputPort<std::string>("obj_name"),
        InputPort<int>("is_exact"),
    };
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("obj_name", request.obj_name);
    int is_exact = 0;
    getInput<int>("is_exact", is_exact);
    request.is_exact = bool(is_exact);
  }
};

class GetPreparePose : public RosServiceNode<roport::GetPreparePose> {
 public:
  GetPreparePose(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::GetPreparePose>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {InputPort<Header>("header"), InputPort<Pose>("pose"), InputPort<int>("is_absolute"),
            InputPort<Point>("shift"), OutputPort<Pose>("pre_pose")};
  }

  void onSendRequest(RequestType& request) override {
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    int is_absolute = 0;
    getInput<int>("is_absolute", is_absolute);
    request.is_absolute = static_cast<bool>(is_absolute);
    Point shift{};
    getInput<Point>("shift", shift);
    request.shift = shift.toROS();
  }

  auto onResponse(const ResponseType& response) -> BT::NodeStatus override {
    if (response.result_status == response.SUCCEEDED) {
      Pose pre_pose{};
      pre_pose.fromROS(response.pre_pose);
      setOutput("pre_pose", pre_pose);
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }
};

class GetTransformedPose : public RosServiceNode<roport::GetTransformedPose> {
 public:
  GetTransformedPose(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::GetTransformedPose>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {InputPort<Header>("header"), InputPort<Pose>("pose"), InputPort<std::string>("source_frame"),
            InputPort<std::string>("target_frame"), OutputPort<Pose>("trans_pose")};
  }

  void onSendRequest(RequestType& request) override {
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
    getInput<std::string>("source_frame", request.source_frame);
    getInput<std::string>("target_frame", request.target_frame);
  }

  auto onResponse(const ResponseType& response) -> BT::NodeStatus override {
    if (response.result_status == response.SUCCEEDED) {
      Pose trans_pose{};
      trans_pose.fromROS(response.trans_pose);
      setOutput("trans_pose", trans_pose);
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }
};

class SenseManipulationPoses : public RosServiceNode<roport::SenseManipulationPoses> {
 public:
  SenseManipulationPoses(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::SenseManipulationPoses>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {InputPort<Header>("header"), InputPort<StringArray>("device_names"), InputPort<int>("algorithm_id"),
            OutputPort<PoseArray>("poses"), OutputPort<Pose>("best_pose")};
  }

  void onSendRequest(RequestType& request) override {
    StringArray device_names{};
    getInput<StringArray>("device_names", device_names);
    request.device_names = device_names.toROS();
    int algorithm_id = -1;
    getInput<int>("algorithm_id", algorithm_id);
    request.algorithm_id = static_cast<uint8_t>(algorithm_id);
  }

  auto onResponse(const ResponseType& response) -> BT::NodeStatus override {
    if (response.result_status == response.SUCCEEDED) {
      PoseArray poses{};
      poses.fromROS(response.poses);
      setOutput("poses", poses);
      Pose best_pose{};
      best_pose.fromROS(response.best_pose);
      setOutput("best_pose", best_pose);
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }
};

class VisualizePose : public RosServiceNode<roport::VisualizePose> {
 public:
  VisualizePose(const ros::NodeHandle& node_handle, const std::string& name, const BT::NodeConfiguration& cfg)
      : RosServiceNode<roport::VisualizePose>(node_handle, name, cfg) {}

  static auto providedPorts() -> BT::PortsList {
    return {InputPort<Header>("header"), InputPort<std::string>("frame"), InputPort<PoseArray>("poses"),
            InputPort<Pose>("pose")};
  }

  void onSendRequest(RequestType& request) override {
    getInput<std::string>("frame", request.frame);
    PoseArray poses{};
    getInput<PoseArray>("poses", poses);
    request.poses = poses.toROS();
    Pose pose{};
    getInput<Pose>("pose", pose);
    request.pose = pose.toROS();
  }
};
}  // namespace bt

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_bt_port");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  std::string tree_file;
  pnh.getParam("tree_file", tree_file);
  if (tree_file.empty()) {
    ROS_ERROR("RoPort: No valid tree file.");
    return -1;
  }

  BT::BehaviorTreeFactory factory;
  BT::registerRosService<bt::ExecuteAddCollisionBox>(factory, "ExecuteAddCollisionBox", node_handle);
  BT::registerRosService<bt::ExecuteAddCollisionPlane>(factory, "ExecuteAddCollisionPlane", node_handle);
  BT::registerRosService<bt::ExecuteAllPlans>(factory, "ExecuteAllPlans", node_handle);
  BT::registerRosService<bt::ExecuteAllPoses>(factory, "ExecuteAllPoses", node_handle);
  BT::registerRosService<bt::ExecuteMirroredPose>(factory, "ExecuteMirroredPose", node_handle);
  BT::registerRosService<bt::ExecuteAttachCollisionBox>(factory, "ExecuteAttachCollisionBox", node_handle);
  BT::registerRosService<bt::ExecuteBinaryAction>(factory, "ExecuteBinaryAction", node_handle);
  BT::registerRosService<bt::ExecuteDetachCollision>(factory, "ExecuteDetachCollision", node_handle);
  BT::registerRosService<bt::ExecuteFrankaGripperGrasp>(factory, "ExecuteFrankaGripperGrasp", node_handle);
  BT::registerRosService<bt::ExecuteGroupAngularJointStates>(factory, "ExecuteGroupAngularJointStates", node_handle);
  BT::registerRosService<bt::ExecuteGroupLinearJointStates>(factory, "ExecuteGroupLinearJointStates", node_handle);
  BT::registerRosService<bt::ExecuteGroupNamedStates>(factory, "ExecuteGroupNamedStates", node_handle);
  BT::registerRosService<bt::ExecuteGroupPlan>(factory, "ExecuteGroupPlan", node_handle);
  BT::registerRosService<bt::ExecuteGroupPose>(factory, "ExecuteGroupPose", node_handle);
  BT::registerRosService<bt::ExecutePathPlanning>(factory, "ExecutePathPlanning", node_handle);
  BT::registerRosService<bt::ExecuteManipulationPlanning>(factory, "ExecuteManipulationPlanning", node_handle);
  BT::registerRosService<bt::ExecuteGroupPosition>(factory, "ExecuteGroupPosition", node_handle);
  BT::registerRosService<bt::ExecuteGroupShift>(factory, "ExecuteGroupShift", node_handle);
  BT::registerRosService<bt::ExecuteJointPosition>(factory, "ExecuteJointPosition", node_handle);
  BT::registerRosService<bt::ExecuteRemoveCollision>(factory, "ExecuteRemoveCollision", node_handle);
  BT::registerRosService<bt::GetPreparePose>(factory, "GetPreparePose", node_handle);
  BT::registerRosService<bt::GetTransformedPose>(factory, "GetTransformedPose", node_handle);
  BT::registerRosService<bt::SenseManipulationPoses>(factory, "SenseManipulationPoses", node_handle);
  BT::registerRosService<bt::VisualizePose>(factory, "VisualizePose", node_handle);

  auto tree = factory.createTreeFromFile(tree_file);
  BT::PublisherZMQ publisher_zmp(tree);

  bt::RosoutLogger logger(tree.rootNode());
  printTreeRecursively(tree.rootNode());

  ROS_WARN("Get ready and press Enter to run the task.");
  std::cin.get();

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  const double kSleep = 0.01;
  while (ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.tickRoot();
    ros::Duration(kSleep).sleep();
  }

  return 0;
}