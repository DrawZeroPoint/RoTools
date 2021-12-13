#include <ros/console.h>
#include <ros/ros.h>

#include "roport/msg_converter.h"

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
