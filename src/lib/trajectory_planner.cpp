#include "roport/trajectory_planner.h"

namespace roport {

CartesianTrajectoryPlanner::CartesianTrajectoryPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  // Get all available planning group names
  XmlRpc::XmlRpcValue group_names;
  roport::getParam(nh_, pnh_, "group_names", group_names);
  ROS_ASSERT(group_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (group_names.size() == 0) {
    throw std::runtime_error("Param group_names not defined");
  }
  for (int i = 0; i < group_names.size(); i++) {
    ROS_ASSERT(group_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string name = std::string(group_names[i]);
    declared_group_names_.push_back(name);

    auto get_pose_client = nh_.serviceClient<roport::GetGroupPose>("/" + name + "/get_group_pose");
    if (!ros::service::waitForService("/" + name + "/get_group_pose", wait_for_service_timeout_)) {
      ROS_WARN("Service %s/get_group_pose is not up, so the client is not created", name.c_str());
      continue;
    }

    group_names_.push_back(name);
    get_current_pose_clients_.push_back(get_pose_client);
  }

  execute_all_cartesian_trajectory_srv_ = nh_.advertiseService(
      "execute_all_cartesian_trajectory", &CartesianTrajectoryPlanner::executeAllCartesianTrajectoryCb, this);
}

auto CartesianTrajectoryPlanner::executeAllCartesianTrajectoryCb(
    roport::ExecuteAllCartesianTrajectories::Request& req,
    roport::ExecuteAllCartesianTrajectories::Response& resp) -> bool {
  if (req.group_names.empty()) {
    ROS_WARN("The 'group_names' in the execute all cartesian trajectory request is empty.");
    resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
    resp.result_msg = "Empty group_names";
    return true;
  }

  //  std::vector<std::pair<std::shared_ptr<CT_Client>, CT_Goal>> goal_temp;
  for (size_t i = 0; i < req.group_names.size(); ++i) {
    auto result = roport::findInVector(group_names_, req.group_names[i]);
    if (!result.first) {
      auto is_in_declared = roport::findInVector(declared_group_names_, req.group_names[i]);
      if (is_in_declared.first) {
        ROS_WARN("Client for group %s was not created, its trajectory will not be executed",
                 req.group_names[i].c_str());
      } else {
        ROS_WARN("Group name %s is not in known names, its trajectory will not be executed",
                 req.group_names[i].c_str());
      }
      continue;
    }

    auto index = result.second;
    auto reference_frame = req.trajectories[i].ref_frame;
    auto ee_frame = req.trajectories[i].ee_frame;

    roport::GetGroupPose get_current_pose;
    get_current_pose.request.ref_frame = reference_frame;
    get_current_pose.request.ee_frame = ee_frame;
    // Use the current pose as the initial commanded pose of the generated cartesian trajectory.
    get_current_pose_clients_[index].call(get_current_pose);

    if (get_current_pose.response.result_status == roport::GetGroupPoseResponse::SUCCEEDED) {
      roport::CartesianTrajectory goal_trajectory;
      CartesianTrajectoryPlanner::makeCartesianTrajectoryWithDrake(get_current_pose.response.pose,
                                                                   req.trajectories[i].points, goal_trajectory);
    } else {
      ROS_WARN("Fail to get the current pose for group %s", group_names_[index].c_str());
      resp.result_status = roport::ExecuteAllCartesianTrajectoriesResponse::FAILED;
      resp.result_msg = "Get current pose failed";
    }
  }

  //  for (const auto& pair : goal_temp) {
  //    pair.first->sendGoal(pair.second, doneCb, activeCb, feedbackCb);
  //  }
  // TODO dzp check whether waiting is necessary?
  //  for (int j = 0; j <= goal_trajectory.trajectory.points.size(); ++j) {
  //    ros::Duration(default_time_step_).sleep();
  //  }
  resp.result_status = resp.SUCCEEDED;
  return true;
}

void CartesianTrajectoryPlanner::geometryPoseToDrakeRigidTransform(const geometry_msgs::Pose& p,
                                                                   drake::math::RigidTransformd& t) {
  Eigen::Matrix4d m;
  geometryPoseToEigenMatrix(p, m);
  t = drake::math::RigidTransformd(m);
}

void CartesianTrajectoryPlanner::drakeRigidTransformToGeometryPose(const drake::math::RigidTransformd& t,
                                                                   geometry_msgs::Pose& p) {
  Eigen::Matrix4d m = t.GetAsMatrix4();
  eigenMatrixToGeometryPose(m, p);
}

void CartesianTrajectoryPlanner::drakeRigidTransformToCartesianTrajectoryPoint(const drake::math::RigidTransformd& pose,
                                                                               const Eigen::Matrix<double, 6, 1>& vel,
                                                                               const Eigen::Matrix<double, 6, 1>& acc,
                                                                               const double& duration,
                                                                               roport::CartesianTrajectoryPoint& p) {
  drakeRigidTransformToGeometryPose(pose, p.pose);
  eigenMatrixToGeometryTwist(vel, p.twist);
  eigenMatrixToGeometryAccel(acc, p.acceleration);
  p.duration = duration;
}

void CartesianTrajectoryPlanner::drakeTrajectoryToCartesianTrajectory(
    const drake::trajectories::PiecewisePose<double>& drake_trajectory,
    roport::CartesianTrajectory& trajectory) const {
  ROS_WARN_STREAM(drake_trajectory.start_time() << "  " << drake_trajectory.end_time());
  int trajectory_length = static_cast<int>(drake_trajectory.end_time() / default_time_step_);
  for (int i = 0; i < trajectory_length; ++i) {
    auto time_stamp = default_time_step_ * i;
    auto pose = drake_trajectory.GetPose(time_stamp);
    auto vel = drake_trajectory.GetVelocity(time_stamp);
    auto acc = drake_trajectory.GetAcceleration(time_stamp);
    roport::CartesianTrajectoryPoint point;
    if (i == 0) {
      drakeRigidTransformToCartesianTrajectoryPoint(pose, vel, acc, 0.0, point);
    } else {
      drakeRigidTransformToCartesianTrajectoryPoint(pose, vel, acc, default_time_step_, point);
    }
    trajectory.points.push_back(point);
  }
}

bool CartesianTrajectoryPlanner::makeCartesianTrajectoryWithDrake(
    geometry_msgs::Pose initial_pose,
    const std::vector<roport::CartesianTrajectoryPoint>& trajectory_points,
    roport::CartesianTrajectory& goal_trajectory) {
  std::vector<double> times;
  std::vector<drake::math::RigidTransformd> poses;

  times.push_back(0.0);  // The time_stamp for the initial_pose
  drake::math::RigidTransformd initial_t;
  geometryPoseToDrakeRigidTransform(initial_pose, initial_t);
  poses.push_back(initial_t);

  for (const auto& p : trajectory_points) {
    times.push_back(p.duration);
    drake::math::RigidTransformd intermediate_t;
    geometryPoseToDrakeRigidTransform(p.pose, intermediate_t);
    poses.push_back(intermediate_t);
  }

  auto drake_trajectory = drake::trajectories::PiecewisePose<double>::MakeLinear(times, poses);
  drakeTrajectoryToCartesianTrajectory(drake_trajectory, goal_trajectory);
  return true;
}
}  // namespace roport
