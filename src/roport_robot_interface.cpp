#include <ros/ros.h>

#include "roport/robot_interface.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_robot_interface");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  roport::RobotInterface robot_interface(nh, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
