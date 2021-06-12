//
// Created by dzp on 2020/10/14.
//

#include "roport/control_server.h"

namespace roport {

  ControlServer::ControlServer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh)
  {
    // Get all available planning group names
    XmlRpc::XmlRpcValue group_names;
    pnh_.getParam("group_names", group_names);
    ROS_ASSERT(group_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

    if (group_names.size() == 0) {
      throw std::runtime_error("RoPort: Move group name list is None");
    }
    for (int i = 0; i < group_names.size(); i++) {
      ROS_ASSERT(group_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      group_names_.push_back(group_names[i]);
    }

    // Get global reference frame for all groups
    std::string reference_frame;
    pnh_.getParam("reference_frame", reference_frame);

    // Initialize planning groups
    for (auto & group_name : group_names_) {
      ROS_INFO_STREAM("RoPort: Initializing move group interface " << group_name);
      auto group = std::make_shared<MoveGroupInterface>(group_name);
      if (!reference_frame.empty()) {
        group->setPoseReferenceFrame(reference_frame);
      }
      interfaces_.push_back(group);
    }

    // Initialize controller action clients, one for each group
    for (auto & group_name : group_names_) {
      std::string action_name = group_name + "_controller/follow_joint_trajectory";
      auto c = std::make_shared<FJT_actionClient>(action_name);
      if (!c->waitForServer(ros::Duration(10))) {
        throw std::runtime_error("RoPort: Action server " + action_name + " unavailable");
      }
      control_clients_.push_back(c);
      ROS_INFO_STREAM("RoPort: Controller action client " << action_name << " initialized");
    }

    // Initialize control servers
    execute_all_poses_srv_ = nh_.advertiseService(
      "execute_all_poses", &ControlServer::executeAllPosesSrvCb, this
    );
    execute_mirrored_pose_srv_ = nh_.advertiseService(
      "execute_mirrored_pose", &ControlServer::executeMirroredPoseSrvCb, this
    );
  }

  ControlServer::~ControlServer()
  {

  }

  bool ControlServer::executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req,
                                           roport::ExecuteAllPoses::Response& resp)
  {
    ROS_ASSERT(req.group_names.size() == req.goals.poses.size());
    std::map<int, trajectory_msgs::JointTrajectory> trajectories;

    for (int i = 0; i < group_names_.size(); ++i) {
      for (int j = 0; j < req.group_names.size(); ++j) {
        if (req.group_names[j] == group_names_[i]) {
          trajectory_msgs::JointTrajectory trajectory;
          geometry_msgs::PoseStamped current_pose_wrt_base = interfaces_[i]->getCurrentPose();
          geometry_msgs::PoseStamped goal_pose_wrt_base;
          goal_pose_wrt_base.header = current_pose_wrt_base.header;

          // Transfer given pose to base frame
          if (req.goal_type == req.BASE_ABS) {
            goal_pose_wrt_base.pose = req.goals.poses[j];
          } else if (req.goal_type == req.BASE_REL) {
            geometry_msgs::PoseStamped transform_wrt_local_base;
            transform_wrt_local_base.pose = req.goals.poses[j];
            relativePoseToAbsolutePose(transform_wrt_local_base, current_pose_wrt_base, goal_pose_wrt_base);
          } else {
            geometry_msgs::PoseStamped goal_pose_wrt_eef;
            goal_pose_wrt_eef.pose = req.goals.poses[j];
            goal_pose_wrt_eef.header.frame_id = interfaces_[i]->getEndEffectorLink();
            goal_pose_wrt_base = tf_buffer_.transform(goal_pose_wrt_eef, interfaces_[i]->getPlanningFrame(), ros::Duration(1));
          }

          // Build trajectory to reach the goal
          bool ok = buildTrajectory(*interfaces_[i], goal_pose_wrt_base, trajectory);
          if (!ok) {
            ROS_ERROR("Building trajectory failed");
            resp.result_status = resp.FAILED;
            return true;
          }
          if (req.stamps.size() == req.group_names.size() && req.stamps[j] > 0) {
            trajectory_msgs::JointTrajectory updated_trajectory;
            updateTrajectoryStamp(trajectory, req.stamps[j], updated_trajectory);
            trajectories.insert({i, updated_trajectory});
          } else {
            trajectories.insert({i, trajectory});
          }
        }
      }
    }
    if (executeTrajectories(trajectories)) {
      resp.result_status = resp.SUCCEEDED;
    } else {
      resp.result_status = resp.FAILED;
    }
    return true;
  }

  bool ControlServer::executeMirroredPoseSrvCb(roport::ExecuteMirroredPose::Request& req,
                                               roport::ExecuteMirroredPose::Response& resp)
  {
    int i = getGroupIndex(req.reference_group);
    if (i < 0) {
      throw std::runtime_error("Reference group is not exist");
    }
    int j = getGroupIndex(req.mirror_group);
    if (j < 0) {
      throw std::runtime_error("Mirror group is not exist");
    }

    std::map<int, trajectory_msgs::JointTrajectory> trajectories;
    trajectory_msgs::JointTrajectory trajectory;
    geometry_msgs::PoseStamped current_pose_wrt_base = interfaces_[i]->getCurrentPose();
    geometry_msgs::PoseStamped goal_pose_wrt_base;
    goal_pose_wrt_base.header = current_pose_wrt_base.header;

    // Transfer given goal pose to the base frame
    if (req.goal_type == req.BASE_ABS) {
      goal_pose_wrt_base.pose = req.goal;
    } else if (req.goal_type == req.BASE_REL) {
      geometry_msgs::PoseStamped transform_wrt_local_base;
      transform_wrt_local_base.pose = req.goal;
      relativePoseToAbsolutePose(transform_wrt_local_base, current_pose_wrt_base, goal_pose_wrt_base);
    } else {
      geometry_msgs::PoseStamped goal_pose_wrt_eef;
      goal_pose_wrt_eef.pose = req.goal;
      goal_pose_wrt_eef.header.frame_id = interfaces_[i]->getEndEffectorLink();
      goal_pose_wrt_base = tf_buffer_.transform(goal_pose_wrt_eef, interfaces_[i]->getPlanningFrame(), ros::Duration(1));
    }

    // Build trajectory to reach the goal
    bool ok = buildTrajectory(*interfaces_[i], goal_pose_wrt_base, trajectory, req.is_cartesian);
    if (!ok) {
      ROS_ERROR("RoPort: Building trajectory failed");
      resp.result_status = resp.FAILED;
      return true;
    }

    trajectory_msgs::JointTrajectory updated_trajectory;
    if (req.stamp > 0) {
      updateTrajectoryStamp(trajectory, req.stamp, updated_trajectory);
    } else {
      updated_trajectory = trajectory;
    }
    trajectories.insert({i, updated_trajectory});

    trajectory_msgs::JointTrajectory mirrored_trajectory;
    getMirroredTrajectory(*interfaces_[j], updated_trajectory, req.mirror_vector, mirrored_trajectory);
    trajectories.insert({j, mirrored_trajectory});

    if (executeTrajectories(trajectories)) {
      resp.result_status = resp.SUCCEEDED;
    } else {
      resp.result_status = resp.FAILED;
    }
    return true;
  }

  void ControlServer::relativePoseToAbsolutePose(geometry_msgs::PoseStamped transform_wrt_local_base,
                                                 geometry_msgs::PoseStamped current_pose_wrt_base,
                                                 geometry_msgs::PoseStamped &goal_pose_wrt_base) {
    // T_be
    Eigen::Matrix4d T_be;
    geometryPoseToEigenMatrix(current_pose_wrt_base, T_be);
    // T_bl
    Eigen::Matrix4d T_bl = Eigen::Matrix4d::Identity();
    Eigen::Vector3d t_bl(current_pose_wrt_base.pose.position.x, current_pose_wrt_base.pose.position.y, current_pose_wrt_base.pose.position.z);
    T_bl.topRightCorner(3, 1) = t_bl;
    // T_le
    Eigen::Matrix4d T_lb = T_bl.inverse();
    Eigen::Matrix4d T_le = T_lb * T_be;
    // T_ll_new
    Eigen::Matrix4d T_ll_new;
    geometryPoseToEigenMatrix(transform_wrt_local_base, T_ll_new);
    // T_be_new
    Eigen::Matrix4d T_be_new = T_bl * T_ll_new * T_le;
    eigenMatrixToGeometryPose(T_be_new, goal_pose_wrt_base);
  }

  void ControlServer::getMirroredTrajectory(MoveGroupInterface &move_group, trajectory_msgs::JointTrajectory trajectory,
                                            std::vector<double> mirror_vector, trajectory_msgs::JointTrajectory &mirrored_trajectory) {
    mirrored_trajectory.joint_names = move_group.getActiveJoints();
    mirrored_trajectory.header = trajectory.header;
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      trajectory_msgs::JointTrajectoryPoint p;
      p = trajectory.points[i];
      ROS_ASSERT(p.positions.size() == mirror_vector.size());
      for (size_t j = 0; j < p.positions.size(); ++j) {
        p.positions[j] *= mirror_vector[j];
        if (!p.velocities.empty()) p.velocities[j] *= mirror_vector[j];
        if (!p.accelerations.empty()) p.accelerations[j] *= mirror_vector[j];
        if (!p.effort.empty()) p.effort[j] *= mirror_vector[j];
      }
      mirrored_trajectory.points.push_back(p);
    }
  }

  bool ControlServer::buildTrajectory(MoveGroupInterface &move_group, const geometry_msgs::PoseStamped& pose,
                                      trajectory_msgs::JointTrajectory& trajectory, bool do_cartesian)
  {
    moveit_msgs::RobotTrajectory robot_trajectory;
    if (!do_cartesian) {
      move_group.clearPoseTargets();
      move_group.setPoseTarget(pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto r = move_group.plan(plan);
      if (r != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR_STREAM("RoPort: Planning for group " + move_group.getName() +
                         " failed with MoveItErrorCode " + std::to_string(r.val));
        return false;
      }
      robot_trajectory = plan.trajectory_;
    } else {
      const double jump_threshold = 0.0; // No joint-space jump constraint (see moveit_msgs/GetCartesianPath)
      const double eef_step = 0.01;

      std::vector<geometry_msgs::Pose> way_points;
      way_points.push_back(pose.pose);
      double fraction = move_group.computeCartesianPath(way_points, eef_step, jump_threshold, robot_trajectory);
      if (fraction != 1) {
        ROS_ERROR_STREAM("RoPort: Planning for group " + move_group.getName() +
                         " failed with fraction " + std::to_string(fraction));
        return false;
      }
    }
    trajectory = robot_trajectory.joint_trajectory;
    trajectory.joint_names = move_group.getActiveJoints();
    assert(!trajectory.joint_names.empty());
    return true;
  }

  bool ControlServer::buildTrajectory(const std::shared_ptr<MoveGroupInterface>& move_group,
                                      const std::vector<geometry_msgs::PoseStamped>& poses,
                                      const std::vector<ros::Duration>& time_stamps,
                                      trajectory_msgs::JointTrajectory& trajectory)
  {
    if (poses.empty())
      throw std::invalid_argument("RoPort: Received empty pose vector");

    if (poses.size() != time_stamps.size()) {
      throw std::invalid_argument("RoPort: Poses and time_stamps should have the same size, but " +
                                  std::to_string(poses.size()) + " vs " + std::to_string(time_stamps.size()));
    }

    moveit::core::RobotStatePtr robot_state = move_group->getCurrentState();
    std::string group_name = move_group->getName();
    const moveit::core::JointModelGroup* joint_group = move_group->getCurrentState()->getJointModelGroup(group_name);

    std::vector<std::vector<double>> joint_poses;
    for (unsigned int i = 0; i < poses.size(); i++) {
      const geometry_msgs::Pose & p = poses[i].pose;
      std::vector<double> joint_pose;
      bool foundIk = robot_state->setFromIK(joint_group, p, move_group->getEndEffectorLink(), 1.0);
      if (!foundIk)
        throw std::runtime_error("RoPort: followEePoseTrajectory failed, IK solution not found for pose " +
                                 std::to_string(i) + " ( requested pose is ("+ std::to_string(p.position.x)+", "+
                                 std::to_string(p.position.y)+", "+std::to_string(p.position.z)+") ("+
                                 std::to_string(p.orientation.x)+", "+std::to_string(p.orientation.y)+", "+
                                 std::to_string(p.orientation.z)+", "+std::to_string(p.orientation.w)+
                                 ") transformed from "+poses.at(i).header.frame_id+")");
      robot_state->copyJointGroupPositions(joint_group, joint_pose);
      joint_poses.push_back(joint_pose);
    }

    trajectory.joint_names = joint_group->getActiveJointModelNames();
    for(unsigned int i = 0; i < joint_poses.size(); i++) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = joint_poses.at(i);
      point.time_from_start = time_stamps.at(i);
      trajectory.points.push_back(point);
    }
    return true;
  }

  void ControlServer::updateTrajectoryStamp(trajectory_msgs::JointTrajectory traj_in, double stamp,
                                            trajectory_msgs::JointTrajectory &traj_out) {
    traj_out.header = traj_in.header;
    traj_out.joint_names = traj_in.joint_names;

    ros::Duration last_stamp = traj_in.points[traj_in.points.size() - 1].time_from_start;
    double ratio = stamp / last_stamp.toSec();
    for (auto & p : traj_in.points) {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = p.positions;
      point.time_from_start = ros::Duration(p.time_from_start.toSec() * ratio);
      traj_out.points.push_back(point);
    }
  }

  bool ControlServer::executeTrajectories(const std::map<int, trajectory_msgs::JointTrajectory>& trajectories)
  {
    std::vector<control_msgs::FollowJointTrajectoryGoal> goals;
    for (auto & t : trajectories) {
      control_msgs::FollowJointTrajectoryGoal goal;
      buildControllerGoal(t.first, t.second, goal);
      goals.push_back(goal);
    }

    std::vector<std::shared_ptr<FJT_actionClient>> clients;
    for (auto & t : trajectories) {
      std::shared_ptr<FJT_actionClient> client = control_clients_[t.first];
      clients.push_back(client);
    }

    for (size_t i = 0; i < clients.size(); ++i) {
      clients[i]->sendGoal(goals[i]);
    }

    for (size_t i = 0; i < clients.size(); ++i) {
      if (!clients[i]->waitForResult(ros::Duration(120))) {
        ROS_ERROR("RoPort: Trajectory execution timeout");
        return false;
      }
    }
    return true;
  }

  void ControlServer::buildControllerGoal(int group_id, const trajectory_msgs::JointTrajectory& trajectory,
                                          control_msgs::FollowJointTrajectoryGoal &goal)
  {
    goal.goal_time_tolerance = ros::Duration(1);
    goal.goal_tolerance = buildTolerance(group_id, 0.01, 0, 0);
    goal.path_tolerance = buildTolerance(group_id, 0.02, 0, 0);
    goal.trajectory = trajectory;
  }

  std::vector<control_msgs::JointTolerance> ControlServer::buildTolerance(int group_id, double position,
                                                                          double velocity, double acceleration)
  {
    control_msgs::JointTolerance joint_tolerance;
    joint_tolerance.position = position;
    joint_tolerance.velocity = velocity;
    joint_tolerance.acceleration = acceleration;
    int joint_num = interfaces_[group_id]->getActiveJoints().size();
    return std::vector<control_msgs::JointTolerance>(joint_num, joint_tolerance);
  }

  void ControlServer::geometryPoseToEigenMatrix(geometry_msgs::PoseStamped pose, Eigen::Matrix4d &mat) {
    mat = Eigen::Matrix4d::Identity();

    Eigen::Quaterniond orientation;
    orientation.coeffs() << pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w;
    Eigen::Quaterniond orientation_n = orientation.normalized();
    Eigen::Matrix3d R_eigen = orientation_n.toRotationMatrix();
    mat.topLeftCorner(3, 3) = R_eigen;

    Eigen::Vector3d t_eigen;
    t_eigen << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
    mat.topRightCorner(3, 1) = t_eigen;
  }

  void ControlServer::eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::PoseStamped &pose) {
    Eigen::Matrix3d R_eigen = mat.topLeftCorner(3, 3);
    Eigen::Vector3d t_eigen = mat.topRightCorner(3, 1);
    Eigen::Quaterniond q(R_eigen);

    pose.pose.position.x = t_eigen[0];
    pose.pose.position.y = t_eigen[1];
    pose.pose.position.z = t_eigen[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
  }

  int ControlServer::getGroupIndex(std::string group_name) {
    auto it = std::find(group_names_.begin(), group_names_.end(), group_name);
    if (it != group_names_.end()) {
      return it - group_names_.begin();
    } else {
      return -1;
    }
  }
}
