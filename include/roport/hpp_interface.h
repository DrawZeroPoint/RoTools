#ifndef HPP_INTERFACE_H
#define HPP_INTERFACE_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <roport/ExecutePathPlanning.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/corbaserver/robot-idl.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/plugin.hh>
#include <hpp/core/problem-solver.hh>

#include "common.h"

using namespace hpp::pinocchio;
using namespace hpp::core;

namespace roport {
class HumanoidPathPlannerInterface {
 public:
  HumanoidPathPlannerInterface(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~HumanoidPathPlannerInterface() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ProblemSolverPtr_t solver_;
  DevicePtr_t robot_;
  DevicePtr_t obstacle_;

  Configuration_t q_init_;

  ros::Subscriber state_subscriber_;
  ros::Subscriber location_subscriber_;

  ros::ServiceServer set_initial_srv_;
  ros::ServiceServer execute_path_planning_srv_;

  bool is_initial_state_set_;
  bool is_initial_location_set_;

  bool createRobot();
  bool setBound();
  bool createObstacle();
  bool setJointConfig(const std::string& joint_name, const double& joint_value, Configuration_t& config);

  static void setLocation(const nav_msgs::Odometry::ConstPtr& msg, Configuration_t& config);

  static bool setLocation(const geometry_msgs::Pose& msg, const int& type, Configuration_t& config);

  void initialJointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

  void initialLocationCb(const nav_msgs::Odometry::ConstPtr& msg);

  auto setInitialSrvCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp) -> bool;

  auto executePathPlanningSrvCb(roport::ExecutePathPlanning::Request& req, roport::ExecutePathPlanning::Response& resp)
      -> bool;

  // Configuration
  std::vector<std::string> joint_names_;
  std::size_t num_joints_;
};

}  // namespace roport

#endif  // HPP_INTERFACE_H
