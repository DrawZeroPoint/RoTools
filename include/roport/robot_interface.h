#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <mutex>
#include <boost/bind.hpp>

using namespace hardware_interface;

namespace roport
{
  class RobotInterface: public hardware_interface::RobotHW
  {
  public:
    RobotInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~RobotInterface() override;

    void init();
    void update(const ros::TimerEvent& e);
    void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

  private:
    void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg, const std::string &topic);

    void sendJointPositionCmd(const ros::Publisher& setter, const std::vector<std::string>& js_names);

    std::vector<std::vector<std::string> > active_joint_groups_;
    std::map<std::string, ros::Subscriber> js_getters_;

    std::vector<ros::Publisher> js_setters_;

    bool received_first_state_;
    size_t received_num_;

  protected:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Timer non_realtime_loop_;

    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // hardware interfaces
    JointStateInterface joint_state_interface_;
    PositionJointInterface q_cmd_interface_;
    VelocityJointInterface dq_cmd_interface_;
    EffortJointInterface tau_J_cmd_interface_;

    // For each joint, this map contains its joint q, dq, and tau_J
    std::map<std::string, std::vector<double>> joint_states_;

    std::timed_mutex joint_states_mutex_;

    // Configuration
    std::vector<std::string> joint_names_;
    std::size_t num_joints_;

    // States
    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> tau_J_;

    // Commands
    std::vector<double> q_cmd_;
    std::vector<double> dq_cmd_;
    std::vector<double> tau_J_cmd_;
  };

}


#endif //ROBOT_INTERFACE_H
