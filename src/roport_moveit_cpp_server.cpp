#include <ros/ros.h>

#include "roport/moveit_cpp_server.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_moveit_cpp_server");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::ControlServer control_server(node_handle, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
