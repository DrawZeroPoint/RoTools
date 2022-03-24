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

#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/manipulation-planner.hh>
#include <hpp/manipulation/problem-solver.hh>

#include <hpp/constraints/fwd.hh>

#include <cmath>
#include <mutex>

#include "common.h"

using namespace hpp::pinocchio;
using namespace hpp::core;

namespace hpp_core = hpp::core;
namespace hpp_pin = hpp::pinocchio;
namespace hpp_man = hpp::manipulation;
namespace hpp_cons = hpp::constraints;

namespace roport {

struct RootJoint {
  static constexpr int kPlanarDOF = 3;
  static constexpr int kPlanarConfigDim = 4;
  static constexpr int kPlanarPositionConfigDim = 2;                                               // p_x, p_y
  static constexpr int kPlanarOrientationConfigDim = kPlanarConfigDim - kPlanarPositionConfigDim;  // ori_w, ori_z

  auto getDOF(const std::string& name) -> int {
    if (name == planar_joint_name) {
      return kPlanarDOF;
    }
  }

  auto getConfigDim(const std::string& name) -> int {
    if (name == planar_joint_name) {
      return kPlanarConfigDim;
    }
  }
  auto getPositionConfigDim(const std::string& name) -> int {
    if (name == planar_joint_name) {
      return kPlanarPositionConfigDim;
    }
  }
  auto getOrientationConfigDim(const std::string& name) -> int {
    if (name == planar_joint_name) {
      return kPlanarOrientationConfigDim;
    }
  }

 private:
  const std::string fixed_joint_name = "anchor";
  const std::string planar_joint_name = "planar";
  const std::string float_joint_name = "freeflyer";
};

class PathPlanningInterface {
 public:
  PathPlanningInterface(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~PathPlanningInterface() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hpp_core::ProblemSolverPtr_t path_planning_solver_;

  hpp_core::DevicePtr_t robot_;
  hpp_core::DevicePtr_t obstacle_;

  RootJoint root_joint_;
  std::string root_joint_type_;

  hpp_core::Configuration_t q_current_;
  hpp_core::Configuration_t q_goal_;

  double position_tolerance_;
  double orientation_tolerance_;

  ros::Subscriber current_state_subscriber_;
  ros::Subscriber current_location_subscriber_;

  ros::ServiceServer execute_path_planning_srv_;

  ros::Publisher joint_command_publisher_;
  ros::Publisher base_vel_cmd_publisher_;

  bool is_initial_state_set_;
  bool is_initial_location_set_;

  // Configuration
  std::map<std::string, std::pair<int, int>> joint_names_;

  static constexpr int kRootJointConfigDim = 4;
  static constexpr int kRootJointPositionConfigDim = 2;

  static constexpr double kDefaultStep = 0.01;  // Time for one step, in second
  static constexpr double kReductionRatio = 0.5;

  static constexpr double kPositionTolerance = 0.01;
  static constexpr double kOrientationTolerance = 0.015;

  static constexpr int kInitializeTimes = 10;
  static constexpr double kInitializeInterval = 0.1;

  double time_step_;

  std::mutex init_mutex_;

  auto createRobot() -> bool;
  auto setBound() -> bool;
  auto createObstacle() -> bool;

  /**
   * Set the config for HPP with the position information from msg.
   * @param msg Msg containing joint names and positions.
   * @param config Output config.
   * @param update_names If true, will update internal joint names, these names will be controlled by the planner.
   *                     Always be true for initial config, false for goal config.
   * @return The number of joints set.
   */
  auto setJointConfig(const sensor_msgs::JointState& msg, hpp_core::Configuration_t& config, const bool& update_names)
      -> size_t;

  auto setLocationConfig(const geometry_msgs::Pose& msg, const int& type, Configuration_t& config) -> bool;

  void currentJointConfigCb(const sensor_msgs::JointState::ConstPtr& msg);

  void currentLocationCb(const nav_msgs::Odometry::ConstPtr& msg);

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

  static void extractBaseVelocityCommand(const Configuration_t& j_q, const vector_t& j_dq, geometry_msgs::Twist& twist);

  void publishPlanningResults(const std::vector<sensor_msgs::JointState>& joint_states,
                              const std::vector<geometry_msgs::Twist>& vel_cmd);

  auto checkGoalLocationReached() -> bool;
};

class ManipulationPlanningInterface {
 public:
  ManipulationPlanningInterface(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh);
  ~ManipulationPlanningInterface() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  hpp_man::ProblemSolverPtr_t manipulation_planning_solver_;

  hpp_man::DevicePtr_t robot_;
  hpp_man::DevicePtr_t obstacle_;

  RootJoint root_joint_;
  std::string root_joint_type_;

  hpp_man::Configuration_t q_current_;
  hpp_man::Configuration_t q_goal_;

  double position_tolerance_;
  double orientation_tolerance_;

  ros::Subscriber current_state_subscriber_;
  ros::Subscriber current_location_subscriber_;

  ros::ServiceServer execute_manipulation_planning_srv_;

  ros::Publisher joint_command_publisher_;
  ros::Publisher base_vel_cmd_publisher_;

  bool is_initial_state_set_;
  bool is_initial_location_set_;

  // Configuration
  std::map<std::string, std::pair<int, int>> joint_names_;

  static constexpr int kMaxIterProjection = 40;

  static constexpr int kRootJointConfigDim = 4;
  static constexpr int kRootJointPositionConfigDim = 2;

  static constexpr double kDefaultStep = 0.01;  // Time for one step, in second
  static constexpr double kReductionRatio = 0.5;

  static constexpr double kPositionTolerance = 0.01;
  static constexpr double kOrientationTolerance = 0.015;

  static constexpr int kInitializeTimes = 10;
  static constexpr double kInitializeInterval = 0.1;

  double time_step_;

  std::mutex init_mutex_;

  auto createRobot() -> bool;
  auto setBound() -> bool;
  auto createObstacle() -> bool;

  /**
   * Set the config for HPP with the position information from msg.
   * @param msg Msg containing joint names and positions.
   * @param config Output config.
   * @param update_names If true, will update internal joint names, these names will be controlled by the planner.
   *                     Always be true for initial config, false for goal config.
   * @return The number of joints set.
   */
  auto setJointConfig(const sensor_msgs::JointState& msg, hpp_core::Configuration_t& config, const bool& update_names)
      -> size_t;

  auto setLocationConfig(const geometry_msgs::Pose& msg, const int& type, Configuration_t& config) -> bool;

  void currentJointConfigCb(const sensor_msgs::JointState::ConstPtr& msg);

  void currentLocationCb(const nav_msgs::Odometry::ConstPtr& msg);

  //  /**
  //   * Check whether the provided config will cause collision.
  //   * @param config Configuration vector.
  //   * @return True if detected collision, false if passed the test.
  //   */
  //  auto detectCollision(const Configuration_t& config) -> bool;
  //
  //  void resetInitialConfig();

  auto executeManipulationPlanningSrvCb(roport::ExecutePathPlanning::Request& req,
                                        roport::ExecutePathPlanning::Response& resp) -> bool;

  //  /**
  //   * Extract joint command from the planning results: the configuration and velocity at time t.
  //   * @param j_q Configuration vector.
  //   * @param j_dq Joint velocity vector.
  //   * @param state Output joint state.
  //   */
  //  void extractJointCommand(const Configuration_t& j_q, const vector_t& j_dq, sensor_msgs::JointState& state);
  //
  //  static void extractBaseVelocityCommand(const Configuration_t& j_q, const vector_t& j_dq, geometry_msgs::Twist&
  //  twist);
  //
  //  void publishPlanningResults(const std::vector<sensor_msgs::JointState>& joint_states,
  //                              const std::vector<geometry_msgs::Twist>& vel_cmd);
  //
  //  auto checkGoalLocationReached() -> bool;
};

}  // namespace roport

#endif  // HPP_INTERFACE_H
