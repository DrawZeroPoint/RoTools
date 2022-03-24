#include <ros/ros.h>

#include "roport/hpp_interface.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_hpp_interface");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::HumanoidPathPlannerInterface hpp_interface(node_handle, pnh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
