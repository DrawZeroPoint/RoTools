//
// Created by dongzhipeng on 3/7/22.
//

#ifndef ROPORT_COMMON_H
#define ROPORT_COMMON_H

#include <ros/ros.h>

namespace roport {

bool getParam(const ros::NodeHandle& node_handle,
              const ros::NodeHandle& private_node_handle,
              const std::string& param_name,
              XmlRpc::XmlRpcValue& param_value) {
  if (!private_node_handle.getParam(param_name, param_value)) {
    if (!node_handle.getParam(param_name, param_value)) {
      ROS_ERROR_STREAM("Param " << param_name << " is not defined");
      return false;
    }
  }
  return true;
}
}  // namespace roport

#endif  // ROPORT_COMMON_H
