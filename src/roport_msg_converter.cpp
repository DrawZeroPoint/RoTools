#include <ros/console.h>
#include <ros/ros.h>

#include <map>

#include "roport/msg_converter.h"

/**
 * Convert sensor_msgs::JointState from one topic to another, the name field could change during the convention
 * To use this function, in the launch file, we need define the following parameters:
 *   source_joint_names_: list[list] The outer list is for all joint groups, each joint group is composed by several
 *                       joints that should be measured together and be controlled by a control topic.
 *   target_joint_names_: list[list] This field have one-on-one correspondence with source_joint_names_, the given joint
 *                       values will be under the new names defined by target_joint_names_.
 *   source_js_topics_: list[str] Joint state topics that should be converted to the target_js_topics_.
 *   target_js_topics_: list[str] Topics converted from source_js_topics_.
 *   target_types_: list[str] Target topic types. By default, the target topics will have the type JointState,
 *                 other types are also supported but need modifying the code. Here we support sensor_msgs/JointState
 *                 and franka_core_msgs/JointCommand. If given, its size must be equal to target_js_topics_.
 *   target_args_: list[int] Give one argument for each target topic publishing. This argument often
 *                defines the control type. If given, its size must be equal to target_js_topics_.
 *   enable_reflex_: list[int] If the value>0, Reflexxes will be used to smooth the source js before conversion.
 *
 *   If enable_reflex_ is set, the following params need to be set:
 *
 *   max_vel: map[str, double] For each joint_name, define its maximum velocity. The name could either be source name
 *            or target name (the same for max_acc and max_jerk).
 *   max_acc: map[str, double] For each joint_name, define its maximum acceleration.
 *   max_jerk: map[str, double] For each joint_name, define its maximum jerk.
 *   scales: map[str, double] The keys should be vel, acc, and jerk; the values are the scale factor to be multiplied
 *           to max_vel, max_acc, and max_jerk, respectively.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "roport_msg_converter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  roport::MsgConverter converter(nh, pnh);

  ROS_INFO("Roport msg converter ready.");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
