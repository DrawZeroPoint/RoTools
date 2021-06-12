//
// Created by dzp on 2020/10/14.
//
#ifndef SRC_CONTROLSERVER_H
#define SRC_CONTROLSERVER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/server/simple_action_server.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <roport/ExecuteAllPoses.h>
#include <roport/ExecuteMirroredPose.h>

using namespace moveit::planning_interface;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FJT_actionClient;

namespace roport
{
  class ControlServer
  {
  public:
    ControlServer(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ControlServer();

  protected:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

  private:
    std::vector<std::string> group_names_;
    std::vector<std::shared_ptr<MoveGroupInterface> > interfaces_;
    tf2_ros::Buffer tf_buffer_;

    ros::ServiceServer execute_all_poses_srv_;
    ros::ServiceServer execute_mirrored_pose_srv_;

    std::vector<std::shared_ptr<FJT_actionClient> > control_clients_;

    bool executeAllPosesSrvCb(roport::ExecuteAllPoses::Request& req, roport::ExecuteAllPoses::Response& resp);
    bool executeMirroredPoseSrvCb(roport::ExecuteMirroredPose::Request& req, roport::ExecuteMirroredPose::Response& resp);

    /**
     *
     * @param move_group The move group to build the trajectory for
     * @param pose Target pose of the trajectory
     * @param trajectory Output trajectory
     * @param do_cartesian
     * @return
     */
    bool buildTrajectory(MoveGroupInterface& move_group, const geometry_msgs::PoseStamped& pose,
                         trajectory_msgs::JointTrajectory& trajectory, bool do_cartesian = false);

    bool buildTrajectory(const std::shared_ptr<MoveGroupInterface>& move_group,
                         const std::vector<geometry_msgs::PoseStamped>& poses,
                         const std::vector<ros::Duration>& time_stamps,
                         trajectory_msgs::JointTrajectory& trajectory);

    void updateTrajectoryStamp(trajectory_msgs::JointTrajectory traj_in, double stamp,
                               trajectory_msgs::JointTrajectory &traj_out);

    bool executeTrajectories(const std::map<int, trajectory_msgs::JointTrajectory>& trajectories);

    void buildControllerGoal(int group_id, const trajectory_msgs::JointTrajectory& trajectory,
                             control_msgs::FollowJointTrajectoryGoal &goal);

    std::vector<control_msgs::JointTolerance> buildTolerance(int group_id, double position,
                                                             double velocity, double acceleration);

    void relativePoseToAbsolutePose(geometry_msgs::PoseStamped transform_wrt_local_base,
                                    geometry_msgs::PoseStamped current_pose_wrt_base,
                                    geometry_msgs::PoseStamped &goal_pose_wrt_base);
    void getMirroredTrajectory(MoveGroupInterface &move_group, trajectory_msgs::JointTrajectory trajectory,
                               std::vector<double> mirror_vector, trajectory_msgs::JointTrajectory &mirrored_trajectory);

    void geometryPoseToEigenMatrix(geometry_msgs::PoseStamped pose, Eigen::Matrix4d &mat);
    void eigenMatrixToGeometryPose(Eigen::Matrix4d mat, geometry_msgs::PoseStamped &pose);

    int getGroupIndex(std::string group_name);
  };

}

#endif //SRC_CONTROLSERVER_H
