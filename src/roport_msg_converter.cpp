#include <ros/console.h>
#include <ros/ros.h>

#include "roport/msg_converter.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_msg_converter");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::MsgConverter converter(node_handle, pnh);

  ROS_INFO("Roport Msg Converter ready.");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
