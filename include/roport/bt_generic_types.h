#ifndef SRC_BT_GENERIC_TYPES_H
#define SRC_BT_GENERIC_TYPES_H

#include <cmath>
#include <ctime>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <utility>

// std_msgs/Header
struct Header {
 private:
  uint32_t seq_{};
  ros::Time stamp_;
  std::string frame_id_;

 public:
  [[nodiscard]] inline auto toROS() const -> std_msgs::Header {
    std_msgs::Header ros_header;
    ros_header.seq = this->seq_;
    ros_header.stamp = this->stamp_;
    ros_header.frame_id = this->frame_id_;
    return ros_header;
  }

  inline void fromROS(const std_msgs::Header& ros_header) {
    this->seq_ = ros_header.seq;
    this->stamp_ = ros_header.stamp;
    this->frame_id_ = ros_header.frame_id;
  }
};

// geometry_msgs/Pose2D
struct Pose2D {
 private:
  double pose_x_;
  double pose_y_;
  double theta_;

 public:
  [[nodiscard]] inline auto toROS() const -> geometry_msgs::Pose2D {
    geometry_msgs::Pose2D ros_pose;
    ros_pose.x = this->pose_x_;
    ros_pose.y = this->pose_y_;
    ros_pose.theta = this->theta_;
    return ros_pose;
  }

  inline void fromROS(geometry_msgs::Pose2D ros_pose) {
    this->pose_x_ = ros_pose.x;
    this->pose_y_ = ros_pose.y;
    this->theta_ = ros_pose.theta;
  }

  [[nodiscard]] auto getX() const -> double { return pose_x_; }
  [[nodiscard]] auto getY() const -> double { return pose_y_; }
  [[nodiscard]] auto getTheta() const -> double { return theta_; }

  void setX(double pose_x) { pose_x_ = pose_x; }
  void setY(double pose_y) { pose_y_ = pose_y; }
  void setTheta(double theta) { theta_ = theta; }
};

// geometry_msgs/Point
struct Point {
 private:
  double point_x_;
  double point_y_;
  double point_z_;

 public:
  [[nodiscard]] inline auto toROS() const -> geometry_msgs::Point {
    geometry_msgs::Point ros_point;
    ros_point.x = this->point_x_;
    ros_point.y = this->point_y_;
    ros_point.z = this->point_z_;
    return ros_point;
  }

  inline void fromROS(geometry_msgs::Point ros_point) {
    this->point_x_ = ros_point.x;
    this->point_y_ = ros_point.y;
    this->point_z_ = ros_point.z;
  }

  [[nodiscard]] auto getX() const -> double { return point_x_; }
  [[nodiscard]] auto getY() const -> double { return point_y_; }
  [[nodiscard]] auto getZ() const -> double { return point_z_; }

  void setX(double point_x) { point_x_ = point_x; }
  void setY(double point_y) { point_y_ = point_y; }
  void setZ(double point_z) { point_z_ = point_z; }
};

// geometry_msgs/Pose
struct Pose {
 private:
  double pose_x_;
  double pose_y_;
  double pose_z_;
  double ori_x_;
  double ori_y_;
  double ori_z_;
  double ori_w_;

 public:
  [[nodiscard]] inline auto toROS() const -> geometry_msgs::Pose {
    geometry_msgs::Pose ros_pose;
    ros_pose.position.x = this->pose_x_;
    ros_pose.position.y = this->pose_y_;
    ros_pose.position.z = this->pose_z_;
    ros_pose.orientation.x = this->ori_x_;
    ros_pose.orientation.y = this->ori_y_;
    ros_pose.orientation.z = this->ori_z_;
    ros_pose.orientation.w = this->ori_w_;
    return ros_pose;
  }

  inline void fromROS(geometry_msgs::Pose ros_pose) {
    this->pose_x_ = ros_pose.position.x;
    this->pose_y_ = ros_pose.position.y;
    this->pose_z_ = ros_pose.position.z;
    this->ori_x_ = ros_pose.orientation.x;
    this->ori_y_ = ros_pose.orientation.y;
    this->ori_z_ = ros_pose.orientation.z;
    this->ori_w_ = ros_pose.orientation.w;
  }

  void setPoseX(double pose_x) { pose_x_ = pose_x; }
  void setPoseY(double pose_y) { pose_y_ = pose_y; }
  void setPoseZ(double pose_z) { pose_z_ = pose_z; }
  void setOriX(double ori_x) { ori_x_ = ori_x; }
  void setOriY(double ori_y) { pose_x_ = ori_y; }
  void setOriZ(double ori_z) { pose_x_ = ori_z; }
  void setOriW(double ori_w) { pose_x_ = ori_w; }
};

// geometry_msgs/PoseArray
struct PoseArray {
 private:
  struct Header header_;
  std::vector<Pose> poses_;

 public:
  [[nodiscard]] inline auto toROS() const -> geometry_msgs::PoseArray {
    geometry_msgs::PoseArray ros_poses;
    for (auto pose : this->poses_) {
      geometry_msgs::Pose ros_pose = pose.toROS();
      ros_poses.poses.push_back(ros_pose);
    }
    ros_poses.header = this->header_.toROS();
    return ros_poses;
  }

  inline void fromROS(const geometry_msgs::PoseArray& ros_poses) {
    this->header_.fromROS(ros_poses.header);

    for (auto ros_pose : ros_poses.poses) {
      Pose pose{};
      pose.fromROS(ros_pose);
      this->poses_.push_back(pose);
    }
  }

  void set(Pose pose) { poses_.push_back(pose); }
};

// geometry_msgs/PoseArray[]
struct PoseArrayArray {
 private:
  std::vector<PoseArray> pose_arrays_;

 public:
  [[nodiscard]] inline auto toROS() const -> std::vector<geometry_msgs::PoseArray> {
    std::vector<geometry_msgs::PoseArray> ros_all_poses;
    for (const auto& pose_array : this->pose_arrays_) {
      geometry_msgs::PoseArray ros_poses = pose_array.toROS();
      ros_all_poses.push_back(ros_poses);
    }
    return ros_all_poses;
  }

  inline void fromROS(const std::vector<geometry_msgs::PoseArray>& ros_all_poses) {
    for (const auto& pose_array : ros_all_poses) {
      PoseArray poses{};
      poses.fromROS(pose_array);
      this->pose_arrays_.push_back(poses);
    }
  }

  void set(const PoseArray& pose_array) { pose_arrays_.push_back(pose_array); }
};

// float64[]
struct DoubleArray {
 private:
  std::vector<double> values_;

 public:
  /**
   * Directly convert the values_ into ROS float64[] message.
   * @return
   */
  [[nodiscard]] inline auto plainToROS() const -> std::vector<double> {
    std::vector<double> output;
    for (auto var : values_) {
      output.push_back(var);
    }
    return output;
  }

  /**
   * Convert the values_ representing degrees into ROS float64[] message for radians.
   * @return
   */
  [[nodiscard]] inline auto degToROS() const -> std::vector<double> {
    std::vector<double> output;
    const double kMultiplier = M_PI / 180.;
    for (auto var : values_) {
      output.push_back(var * kMultiplier);
    }
    return output;
  }

  inline void fromROS(std::vector<double> var) { this->values_ = std::move(var); }

  void set(const double& var) { values_.push_back(var); }
};

// string[]
struct StringArray {
 private:
  std::vector<std::string> values_;

 public:
  [[nodiscard]] inline auto toROS() const -> std::vector<std::string> {
    std::vector<std::string> output;
    for (const auto& var : values_) {
      output.push_back(var);
    }
    return output;
  }

  inline void fromROS(std::vector<std::string> var) { this->values_ = std::move(var); }

  void set(const std::string& var) { values_.push_back(var); }
};

/**
 * Note that BT::convertFromString() do not support template specialization of float,
 * so we use double for all the float values.
 */
namespace BT {
template <>
inline auto convertFromString(StringView str) -> Pose2D {
  // We expect real numbers separated by spaces
  auto parts = splitString(str, ' ');
  if (parts.size() != 3) {
    throw RuntimeError("Invalid input '%s' for Pose2D", str);
  }
  Pose2D output{};
  output.setX(convertFromString<double>(parts[0]));
  output.setY(convertFromString<double>(parts[1]));
  output.setTheta(convertFromString<double>(parts[2]));
  return output;
}

template <>
inline auto convertFromString(StringView str) -> Point {
  // We expect real numbers separated by spaces
  auto parts = splitString(str, ' ');
  if (parts.size() != 3) {
    throw RuntimeError("Invalid input '%s' for Point)", str);
  }
  Point output{};
  output.setX(convertFromString<double>(parts[0]));
  output.setY(convertFromString<double>(parts[1]));
  output.setZ(convertFromString<double>(parts[2]));
  return output;
}

template <>
inline auto convertFromString(StringView str) -> Pose {
  // We expect real numbers separated by spaces
  auto parts = splitString(str, ' ');
  const size_t kPartSize = 7;
  if (parts.size() != kPartSize) {
    throw RuntimeError("Invalid input %s for Pose)", str);
  }
  Pose output{};
  output.setPoseX(convertFromString<double>(parts[0]));
  output.setPoseY(convertFromString<double>(parts[1]));
  output.setPoseZ(convertFromString<double>(parts[2]));
  output.setOriX(convertFromString<double>(parts[3]));
  output.setOriY(convertFromString<double>(parts[4]));
  output.setOriZ(convertFromString<double>(parts[5]));
  output.setOriW(convertFromString<double>(parts[6]));
  return output;
}

template <>
inline auto convertFromString(StringView str) -> PoseArray {
  // We expect poses separated by semicolons
  auto poses_str = splitString(str, ';');
  PoseArray output{};

  for (auto pose_str : poses_str) {
    output.set(convertFromString<Pose>(pose_str));
  }
  return output;
}

template <>
inline auto convertFromString(StringView str) -> PoseArrayArray {
  auto pose_arrays = splitString(str, '|');

  PoseArrayArray output{};
  for (auto pa_str : pose_arrays) {
    output.set(convertFromString<PoseArray>(pa_str));
  }
  return output;
}

template <>
inline auto convertFromString(StringView str) -> DoubleArray {
  // We expect real numbers separated by spaces
  auto parts = splitString(str, ' ');

  DoubleArray output;
  for (auto part : parts) {
    output.set(convertFromString<double>(part));
  }
  return output;
}

template <>
inline auto convertFromString(StringView str) -> StringArray {
  // We expect real numbers separated by spaces
  auto parts = splitString(str, ' ');

  StringArray output;
  for (auto part : parts) {
    output.set(convertFromString<std::string>(part));
  }
  return output;
}

}  // end namespace BT

#endif  // SRC_BT_GENERIC_TYPES_H
