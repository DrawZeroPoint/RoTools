#include <ros/ros.h>

#include "roport/cartesio_server.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_cartesio_server");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::CartesIOServer cartesio_server(node_handle, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
