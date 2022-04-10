//
// Created by Ray on 06/04/2022.
//

#include <ros/console.h>
#include <ros/ros.h>

#include "roport/msg_aggregator.h"

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "roport_msg_aggregator");
  ros::NodeHandle node_handle;
  ros::NodeHandle pnh("~");

  roport::MsgAggregator aggregator(node_handle, pnh);

  ROS_INFO("Roport Msg Aggregator ready.");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
