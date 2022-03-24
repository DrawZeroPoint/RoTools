#include <ros/ros.h>

#include "roport/robot_interface.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_robot_interface");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::RobotInterface robot_interface(node_handle, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
