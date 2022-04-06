//
// Created by Ray on 06/04/2022.
//

#include "roport/msg_aggregator.h"

namespace roport {

MsgAggregator::MsgAggregator(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh) {
  panda_left_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_,"/panda_left/joint_states",1);
  panda_right_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_,"/panda_right/joint_states",1);
  panda_left_finger_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_,"/panda_left/gripper/joint_states",1);
  panda_right_finger_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_,"/panda_right/gripper/joint_states",1);
  curi_head_sub_ = new message_filters::Subscriber<sensor_msgs::JointState >(nh_, "/curi_head/joint_states",1);
  curi_torso_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/curi_torso/joint_states",1);
  joint_states_sync_ = new message_filters::Synchronizer<CuriJointStateSyncPolicy>(CuriJointStateSyncPolicy(10), *curi_head_sub_, *panda_left_sub_,
                                                                      *panda_left_finger_sub_, *panda_right_sub_, *panda_right_finger_sub_, *curi_torso_sub_);
//  joint_states_sync_ = new message_filters::Synchronizer<CuriJointStateSyncPolicy>(CuriJointStateSyncPolicy(10),*panda_left_sub_,*panda_right_sub_);
  joint_states_sync_->registerCallback(boost::bind(&MsgAggregator::curiJointStatesCB,this, _1, _2, _3, _4, _5, _6));
//  joint_states_sync_->registerCallback(boost::bind(&MsgAggregator::curiJointStatesCB,this, _1, _2));
  curi_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/curi/joint_states", 1);
}

void MsgAggregator::curiJointStatesCB(const sensor_msgs::JointState::ConstPtr& curi_head, const sensor_msgs::JointState::ConstPtr& panda_left,
                                      const sensor_msgs::JointState::ConstPtr& panda_left_finger, const sensor_msgs::JointState::ConstPtr& panda_right,
                                      const sensor_msgs::JointState::ConstPtr& panda_right_finger, const sensor_msgs::JointState::ConstPtr& curi_torso) {
  sensor_msgs::JointState curi_current_joint_states;
  curi_current_joint_states.header.frame_id = "test";
  curi_current_joint_states.position[0] = curi_head->position[0];
  curi_current_joint_states.position[1] = curi_head->position[1];
  for (int i = 0; i < 7; ++i) {
    curi_current_joint_states.position[2+i] = panda_left->position[i];
    curi_current_joint_states.position[9+i] = panda_right->position[i];
  }
  curi_current_joint_states.position[16] = panda_left_finger->position[0];
  curi_current_joint_states.position[17] = panda_right_finger->position[0];
  for (int i = 0; i < 3; ++i) {
    curi_current_joint_states.position[18+i] = curi_torso->position[i];
  }
  ROS_WARN("Receive msg and then publish!!");
}

//void MsgAggregator::curiJointStatesCB(const sensor_msgs::JointState::ConstPtr& panda_left, const sensor_msgs::JointState::ConstPtr& panda_right) {
//  sensor_msgs::JointState curi_current_joint_states;
//  curi_current_joint_states.header.frame_id = "test";
//  for (int i = 0; i < 7; ++i) {
//    curi_current_joint_states.position[i] = panda_left->position[i];
//    curi_current_joint_states.position[7+i] = panda_right->position[i];
//  }
//  curi_joint_states_pub_.publish(curi_current_joint_states);
//
//  ROS_WARN("Receive msg and then publish!!");
//}

}