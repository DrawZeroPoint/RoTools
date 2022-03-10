//
// Created by dongzhipeng on 3/7/22.
//

#ifndef ROPORT_COMMON_H
#define ROPORT_COMMON_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

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

void geometryPoseToEigenMatrix(const geometry_msgs::Pose& pose, Eigen::Matrix4d& mat) {
  mat = Eigen::Matrix4d::Identity();

  Eigen::Quaterniond orientation;
  orientation.coeffs() << pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w;
  Eigen::Quaterniond orientation_n = orientation.normalized();
  Eigen::Matrix3d rotation_matrix = orientation_n.toRotationMatrix();
  mat.topLeftCorner(3, 3) = rotation_matrix;

  Eigen::Vector3d t_eigen;
  t_eigen << pose.position.x, pose.position.y, pose.position.z;
  mat.topRightCorner(3, 1) = t_eigen;
}

void geometryPoseToEigenMatrix(const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d& mat) {
  geometryPoseToEigenMatrix(pose.pose, mat);
}

void eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::Pose& pose) {
  Eigen::Matrix3d rotation_matrix = mat.topLeftCorner(3, 3);
  Eigen::Vector3d t_eigen = mat.topRightCorner(3, 1);
  Eigen::Quaterniond quat(rotation_matrix);

  pose.position.x = t_eigen[0];
  pose.position.y = t_eigen[1];
  pose.position.z = t_eigen[2];
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

void eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::PoseStamped& pose) {
  eigenMatrixToGeometryPose(mat, pose.pose);
}

void relativePoseToAbsolutePose(const geometry_msgs::PoseStamped& transform_wrt_local_base,
                                const geometry_msgs::PoseStamped& current_pose_wrt_base,
                                geometry_msgs::PoseStamped& goal_pose_wrt_base) {
  Eigen::Matrix4d mat_be;
  geometryPoseToEigenMatrix(current_pose_wrt_base, mat_be);
  // mat_bl
  Eigen::Matrix4d mat_bl = Eigen::Matrix4d::Identity();
  Eigen::Vector3d t_bl(current_pose_wrt_base.pose.position.x, current_pose_wrt_base.pose.position.y,
                       current_pose_wrt_base.pose.position.z);
  mat_bl.topRightCorner(3, 1) = t_bl;
  // mat_le
  Eigen::Matrix4d mat_le = mat_bl.inverse() * mat_be;
  // mat_ll_new
  Eigen::Matrix4d mat_ll_new;
  geometryPoseToEigenMatrix(transform_wrt_local_base, mat_ll_new);
  // mat_be_new
  Eigen::Matrix4d mat_be_new = mat_bl * mat_ll_new * mat_le;
  eigenMatrixToGeometryPose(mat_be_new, goal_pose_wrt_base);
}

void localPoseToGlobalPose(const geometry_msgs::Pose& pose_local_to_target,
                           const geometry_msgs::Pose& pose_global_to_local,
                           geometry_msgs::Pose& pose_global_to_target) {
  Eigen::Matrix4d mat_local_to_target;
  geometryPoseToEigenMatrix(pose_local_to_target, mat_local_to_target);
  Eigen::Matrix4d mat_global_to_local;
  geometryPoseToEigenMatrix(pose_local_to_target, mat_global_to_local);
  Eigen::Matrix4d mat_global_to_target = mat_global_to_local * mat_local_to_target;
  eigenMatrixToGeometryPose(mat_global_to_target, pose_global_to_target);
}

}  // namespace roport

#endif  // ROPORT_COMMON_H
