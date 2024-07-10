#include <ros/ros.h>

#include "roport/trajectory_planner.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_trajectory_planner");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::TrajectoryPlanner planner(node_handle, pnh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
