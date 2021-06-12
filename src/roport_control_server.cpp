#include <ros/ros.h>

#include "roport/control_server.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "roport_control_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  roport::ControlServer control_server(nh, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}


