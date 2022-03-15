#ifndef HPP_INTERFACE_H
#define HPP_INTERFACE_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <roport/ExecuteBinaryAction.h>
#include <roport/ExecutePathPlanning.h>
#include <sensor_msgs/JointState.h>

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

namespace hpp_core = hpp::core;
namespace hpp_pin = hpp::pinocchio;

namespace roport {
class HumanoidPathPlannerInterface {
 public:
  HumanoidPathPlannerInterface(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~HumanoidPathPlannerInterface() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hpp_core::ProblemSolverPtr_t path_planning_solver_;

  hpp_core::DevicePtr_t robot_;
  hpp_core::DevicePtr_t obstacle_;

  hpp_core::Configuration_t q_init_;

  ros::Subscriber state_subscriber_;
  ros::Subscriber location_subscriber_;

  ros::ServiceServer execute_path_planning_srv_;

  ros::Publisher joint_command_publisher_;
  ros::Publisher base_vel_cmd_publisher_;

  bool is_initial_state_set_;
  bool is_initial_location_set_;

  // Configuration
  std::map<std::string, std::pair<int, int>> joint_names_;

  static constexpr int kPlannerDim = 6;
  static constexpr double kDefaultStep = 0.01;

  double time_step_;

  auto createRobot() -> bool;
  auto setBound() -> bool;
  auto createObstacle() -> bool;

  void setJointConfig(const sensor_msgs::JointState& msg, hpp_core::Configuration_t& config, const bool& update_names);

  static void setLocation(const nav_msgs::Odometry::ConstPtr& msg, Configuration_t& config);

  static auto setLocation(const geometry_msgs::Pose& msg, const int& type, Configuration_t& config) -> bool;

  void initialJointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

  void initialLocationCb(const nav_msgs::Odometry::ConstPtr& msg);

  /**
   * Check whether the provided config will cause collision.
   * @param config Configuration vector.
   * @return True if detected collision, false if passed the test.
   */
  auto detectCollision(const Configuration_t& config) -> bool;

  void resetInitialConfig();

  auto executePathPlanningSrvCb(roport::ExecutePathPlanning::Request& req, roport::ExecutePathPlanning::Response& resp)
      -> bool;

  /**
   * Extract joint command from the planning results: the configuration and velocity at time t.
   * @param j_q Configuration vector.
   * @param j_dq Joint velocity vector.
   * @param state Output joint state.
   */
  void extractJointCommand(const Configuration_t& j_q, const vector_t& j_dq, sensor_msgs::JointState& state);

  static void extractBaseVelocityCommand(const vector_t& j_dq, geometry_msgs::Twist& twist);

  void publishPlanningResults(const std::vector<sensor_msgs::JointState>& joint_states,
                              const std::vector<geometry_msgs::Twist>& vel_cmd);
};

}  // namespace roport

#endif  // HPP_INTERFACE_H
