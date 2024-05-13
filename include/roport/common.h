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
#include <cmath>

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
error "Missing the <filesystem> header."
#endif
#include <utility>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

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

inline void geometryPoseToEigen(const geometry_msgs::Pose& pose, Eigen::Vector3d& trans, Eigen::Quaterniond& quat) {
  trans << pose.position.x, pose.position.y, pose.position.z;
  quat.coeffs() << pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w;
}

inline void eigenToGeometryPose(const Eigen::Vector3d& trans,
                                const Eigen::Quaterniond& quat,
                                geometry_msgs::Pose& pose) {
  pose.position.x = trans[0];
  pose.position.y = trans[1];
  pose.position.z = trans[2];
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
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

inline void geometryPoseStampedToEigenMatrix(const geometry_msgs::PoseStamped& pose, Eigen::Matrix4d& mat) {
  geometryPoseToEigenMatrix(pose.pose, mat);
}

inline void geometryTransformToEigenMatrix(const geometry_msgs::Transform& t, Eigen::Matrix4d& mat) {
  mat = Eigen::Matrix4d::Identity();

  Eigen::Quaterniond orientation;
  orientation.coeffs() << t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w;
  Eigen::Quaterniond orientation_n = orientation.normalized();
  Eigen::Matrix3d rotation_matrix = orientation_n.toRotationMatrix();
  mat.topLeftCorner(3, 3) = rotation_matrix;

  Eigen::Vector3d t_eigen;
  t_eigen << t.translation.x, t.translation.y, t.translation.z;
  mat.topRightCorner(3, 1) = t_eigen;
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

inline void toGlobalPose(const int& goal_type,
                         const geometry_msgs::Pose& current_pose,
                         const geometry_msgs::Pose& cmd_pose,
                         geometry_msgs::Pose& goal_pose) {
  if (goal_type == 0) {
    // The given pose is already in the reference frame
    goal_pose = cmd_pose;
  } else if (goal_type == 1) {
    // The given pose is relative to the local aligned frame having the same orientation as the reference frame
    localAlignedPoseToGlobalPose(cmd_pose, current_pose, goal_pose, true);
  } else if (goal_type == 2) {
    // The given pose is relative to the local frame
    localPoseToGlobalPose(cmd_pose, current_pose, goal_pose);
  } else {
    throw std::invalid_argument("Goal type not supported");
  }
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
 * Converting a quaternion representing a planar rotation to theta.
 * @param quat Quaternion msg.
 * @return theta.
 */
inline double quaternionToTheta(const geometry_msgs::Quaternion& quat) {
  Eigen::Quaterniond quaternion(quat.w, quat.x, quat.y, quat.z);
  Eigen::Rotation2Dd rot(quaternion.toRotationMatrix().topLeftCorner<2, 2>());
  return rot.smallestPositiveAngle();
}

/**
 * Pretty print a Eigen matrix. Its size, row number, and column number will also be displayed.
 * @param mat A matrix with various shape to print.
 * @param precision How many digits to keep after the decimal point.
 * @param title An optional title of the output.
 */
inline void prettyPrintEigenMatrix(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& mat,
                                   const int& precision = 3,
                                   const std::string& title = "") {
  Eigen::IOFormat cleanFormat(precision, 0, ", ", "\n", "[", "]");
  ROS_INFO_STREAM("\n"
                  << title << ": Matrix size: " << mat.size() << " rows: " << mat.rows() << " cols: " << mat.cols());
  ROS_INFO_STREAM("\n" << mat.format(cleanFormat));
}

inline void getCartesianError(const Eigen::Vector3d& current_position,
                              const Eigen::Quaterniond& current_orientation,
                              const Eigen::Vector3d& goal_position,
                              const Eigen::Quaterniond& goal_orientation,
                              Eigen::Matrix<double, 6, 1>& error) {
  error.head(3) << current_position - goal_position;

  Eigen::Affine3d current_affine_transform;
  current_affine_transform.translation() = current_position;
  current_affine_transform.linear() = current_orientation.toRotationMatrix();

  Eigen::Quaterniond regularized_current_orientation;
  regularized_current_orientation.coeffs() << current_orientation.coeffs();
  if (goal_orientation.coeffs().dot(current_orientation.coeffs()) < 0.) {
    regularized_current_orientation.coeffs() << -current_orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(regularized_current_orientation.inverse() * goal_orientation);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -current_affine_transform.linear() * error.tail(3);
}

inline void getCartesianError(const Eigen::Affine3d& current_transform,
                              const Eigen::Affine3d& goal_transform,
                              Eigen::Matrix<double, 6, 1>& error) {
  Eigen::Vector3d current_position(current_transform.translation());
  Eigen::Quaterniond current_orientation(current_transform.linear());
  Eigen::Vector3d goal_position(goal_transform.translation());
  Eigen::Quaterniond goal_orientation(goal_transform.linear());
  getCartesianError(current_position, current_orientation, goal_position, goal_orientation, error);
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
    std::cerr << "Tolerance should be positive" << std::endl;
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

///
/// \brief Compute the minimal angle between q1 and q2.
/// github.com/stack-of-tasks/pinocchio/blob/c2ad2c60eecc04555e265a8f80a9765d3b84f5f1/src/math/quaternion.hpp#L35
///
/// \param[in] q1 input quaternion.
/// \param[in] q2 input quaternion.
///
/// \return angle between the two quaternions
///
inline double angleBetweenQuaternions(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
  const double inner_prod = q1.dot(q2);
  double theta = std::acos(inner_prod);
  if (inner_prod < 0) {
    theta = M_PI - theta;
  }
  return theta;
}

inline bool allClose(geometry_msgs::Pose first,
                     geometry_msgs::Pose second,
                     double& position_residual,
                     double& orientation_residual,
                     double position_tolerance = 0.01,
                     double orientation_tolerance = 0.1) {
  bool is_all_close = true;
  position_residual = 0;
  orientation_residual = 0;

  if (fabs(first.position.x - second.position.x) > position_tolerance) {
    is_all_close = false;
  }
  position_residual += fabs(first.position.x - second.position.x);

  if (fabs(first.position.y - second.position.y) > position_tolerance) {
    is_all_close = false;
  }
  position_residual += fabs(first.position.y - second.position.y);

  if (fabs(first.position.z - second.position.z) > position_tolerance) {
    is_all_close = false;
  }
  position_residual += fabs(first.position.z - second.position.z);

  Eigen::Quaterniond q1, q2;
  q1.coeffs() << first.orientation.x, first.orientation.y, first.orientation.z, first.orientation.w;
  q2.coeffs() << second.orientation.x, second.orientation.y, second.orientation.z, second.orientation.w;
  auto theta = angleBetweenQuaternions(q1, q2);
  if (fabs(theta) > orientation_residual) {
    is_all_close = false;
  }
  orientation_residual += fabs(theta);
  return is_all_close;
}

template <typename T>
inline void logWarningList(const T& list, const std::string& title = "List in focus:") {
  ROS_WARN_STREAM(title << " In total " << list.size() << " values");
  for (size_t idx = 0; idx < list.size(); ++idx) {
    ROS_WARN_STREAM("# " << idx << " " << list[idx]);
  }
}

inline auto getFilePath(const std::string& raw_path, std::string& full_path) -> bool {
  if (fs::exists(raw_path)) {
    full_path = raw_path;
    return true;
  } else {
    auto home_dir = getenv("HOME");
    if (home_dir == NULL) {
      return false;
    }
    fs::path path = fs::path(home_dir) / raw_path;
    if (fs::exists(path)) {
      full_path = path;
      return true;
    } else {
      return false;
    }
  }
}

/**
 * Get the null space projector of the secondary task with the jacobian of the primary task.
 * The yield projector is used for torque level control.
 * @param jacobian Jacobian matrix.
 * @retval ns Null space projector matrix.
 * @param e Threshold
 * @sa On continuous null space projections for torque-based, hierarchical, multi-objective manipulation eq. 8 - eq. 10
 * @li https://ieeexplore.ieee.org/document/6224571
 */
inline void getNullSpaceProjector(const Eigen::MatrixXd& jacobian, Eigen::MatrixXd& ns, const double& e = 0.001) {
  auto n = jacobian.cols();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);

  for (int i = 0; i < sing_vals_.size(); i++) {
    if (sing_vals_(i) < e) {
      A(i, i) = 0;
    } else {
      A(i, i) = sing_vals_(i);
    }
  }

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  ns = I - svd.matrixV() * A * svd.matrixV().transpose();
}

inline void clamp(const Eigen::VectorXd& input,
                  const Eigen::VectorXd& upper,
                  const Eigen::VectorXd& lower,
                  Eigen::VectorXd& output) {
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    if (input(i) > upper(i)) {
      output(i) = upper(i);
    } else if (input(i) < lower(i)) {
      output(i) = lower(i);
    } else {
      output(i) = input(i);
    }
  }
}

inline void saturate(const Eigen::VectorXd& input,
                     const Eigen::VectorXd& reference,
                     const Eigen::VectorXd& delta_upper,
                     const Eigen::VectorXd& delta_lower,
                     Eigen::VectorXd& output) {
  output.resize(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    double delta = input[i] - reference[i];
    output[i] = reference[i] + std::max(std::min(delta, delta_upper[i]), delta_lower[i]);
  }
}
}  // namespace roport

#endif  // ROPORT_COMMON_H
