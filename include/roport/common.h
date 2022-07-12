/*
 * common
 * Copyright (c) 2021-2022, Zhipeng Dong
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROPORT_COMMON_H
#define ROPORT_COMMON_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <utility>

#include <geometry_msgs/PoseStamped.h>

namespace roport {

constexpr double kQuaternionOrientationTolerance = 0.01;
constexpr double kTolerance = 0.01;

inline auto getParam(const ros::NodeHandle& node_handle,
                     const ros::NodeHandle& private_node_handle,
                     const std::string& param_name,
                     XmlRpc::XmlRpcValue& param_value) -> bool {
  if (!private_node_handle.getParam(param_name, param_value)) {
    if (!node_handle.getParam(param_name, param_value)) {
      ROS_ERROR_STREAM("Param " << param_name << " is not defined");
      return false;
    }
  }
  return true;
}

inline auto getIndex(const std::vector<std::string>& names, const std::string& target) -> int {
  auto res = std::find(names.begin(), names.end(), target);
  if (res != names.end()) {
    return res - names.begin();
  }
  return -1;
}

inline void geometryPoseToEigenMatrix(const geometry_msgs::Pose& pose, Eigen::Matrix4d& mat) {
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

inline void geometryPoseToEigenMatrix(const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d& mat) {
  geometryPoseToEigenMatrix(pose.pose, mat);
}

inline void eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::Pose& pose) {
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

inline void eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::PoseStamped& pose) {
  eigenMatrixToGeometryPose(std::move(mat), pose.pose);
}

inline void localPoseToGlobalPose(const geometry_msgs::Pose& pose_local_to_target,
                                  const geometry_msgs::Pose& pose_global_to_local,
                                  geometry_msgs::Pose& pose_global_to_target) {
  Eigen::Matrix4d mat_local_to_target;
  geometryPoseToEigenMatrix(pose_local_to_target, mat_local_to_target);
  Eigen::Matrix4d mat_global_to_local;
  geometryPoseToEigenMatrix(pose_global_to_local, mat_global_to_local);
  Eigen::Matrix4d mat_global_to_target = mat_global_to_local * mat_local_to_target;
  eigenMatrixToGeometryPose(mat_global_to_target, pose_global_to_target);
}

/**
 * Given a transform from the local aligned frame to the target frame T_la_t, and a transform
 * from the global frame to the local frame T_g_l, this function derives T_g_t, where
 * the local aligned frame's orientation is identical with the global frame.
 * @param pose_local_aligned_to_target Target pose in the local aligned frame.
 * @param pose_global_to_local Local pose in the global frame.
 * @retval pose_global_to_target Target pose in the global frame.
 * @param copy_orientation If true, will copy the orientation of T_g_l to T_la_t, such that T_la_t's
 *                         orientation is overwritten.
 */
inline void localAlignedPoseToGlobalPose(const geometry_msgs::Pose& pose_local_aligned_to_target,
                                         const geometry_msgs::Pose& pose_global_to_local,
                                         geometry_msgs::Pose& pose_global_to_target,
                                         const bool& copy_orientation = false) {
  geometry_msgs::Pose local_aligned_to_target;
  if (copy_orientation) {
    local_aligned_to_target.position = pose_local_aligned_to_target.position;
    local_aligned_to_target.orientation = pose_global_to_local.orientation;
  } else {
    local_aligned_to_target = pose_local_aligned_to_target;
  }
  geometry_msgs::Pose global_to_local;
  global_to_local.position = pose_global_to_local.position;
  global_to_local.orientation.x = 0.;
  global_to_local.orientation.y = 0.;
  global_to_local.orientation.z = 0.;
  global_to_local.orientation.w = 1.;

  Eigen::Matrix4d mat_local_aligned_to_target;
  geometryPoseToEigenMatrix(local_aligned_to_target, mat_local_aligned_to_target);
  Eigen::Matrix4d mat_global_to_local_aligned;
  geometryPoseToEigenMatrix(global_to_local, mat_global_to_local_aligned);
  Eigen::Matrix4d mat_global_to_target = mat_global_to_local_aligned * mat_local_aligned_to_target;
  eigenMatrixToGeometryPose(mat_global_to_target, pose_global_to_target);
}

inline auto isPoseLegal(const geometry_msgs::Pose& pose) -> bool {
  if (pose.orientation.w == 0 && pose.orientation.x == 0 && pose.orientation.y == 0 && pose.orientation.z == 0) {
    ROS_ERROR("The pose is empty (all orientation coefficients are zero)");
    return false;
  }
  if (fabs(std::pow(pose.orientation.w, 2) + std::pow(pose.orientation.x, 2) + std::pow(pose.orientation.y, 2) +
           std::pow(pose.orientation.z, 2) - 1.) > kQuaternionOrientationTolerance) {
    ROS_WARN("The pose has un-normalized orientation");
    return false;
  }
  return true;
}

/**
 * Judge if all corresponding elements in the two given vectors are close to each other under the tolerance.
 * @tparam T Value type.
 * @param first One vector.
 * @param second Other vector.
 * @param violated_i The index of the first element that violates the tolerance.
 * @param residual The residual of the first violation.
 * @param tol Tolerance.
 * @return True if close.
 */
template <typename T>
inline auto allClose(std::vector<T> first, std::vector<T> second, size_t& violated_i, T& residual, T tol = kTolerance)
    -> bool {
  if (tol <= 0) {
    tol = kTolerance;
  }
  for (size_t i = 0; i < first.size(); ++i) {
    auto error = fabs(first[i] - second[i]);
    if (error > tol) {
      violated_i = i;
      residual = error;
      return false;
    }
  }
  return true;
}

template <typename T>
inline auto allClose(std::vector<T> first, std::vector<T> second, size_t& violated_i, T& residual, std::vector<T> tol)
    -> bool {
  if (tol.size() != first.size() || tol.size() != second.size()) {
    throw std::runtime_error("Vector size mismatch");
  }
  for (size_t i = 0; i < first.size(); ++i) {
    auto error = fabs(first[i] - second[i]);
    if (error > tol[i]) {
      violated_i = i;
      residual = error;
      return false;
    }
  }
  return true;
}

template <typename T>
inline void logWarningList(const T& list, const std::string& title = "List in focus:") {
  ROS_WARN_STREAM(title << " In total " << list.size() << " values");
  for (size_t idx = 0; idx < list.size(); ++idx) {
    ROS_WARN_STREAM("# " << idx << " " << list[idx]);
  }
}
}  // namespace roport

#endif  // ROPORT_COMMON_H
