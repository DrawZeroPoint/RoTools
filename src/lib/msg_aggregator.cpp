//
// Created by Ray on 06/04/2022.
//

#include "roport/msg_aggregator.h"

namespace roport {

MsgAggregator::MsgAggregator(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh) {
  panda_left_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/panda_left/joint_states", 1);
  panda_right_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/panda_right/joint_states", 1);
  panda_left_finger_sub_ =
      new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/panda_left/gripper/joint_states", 1);
  panda_right_finger_sub_ =
      new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/panda_right/gripper/joint_states", 1);
  curi_head_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/curi_head/joint_states", 1);
  curi_torso_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "/curi_torso/joint_states", 1);
  joint_states_sync_ = new message_filters::Synchronizer<CuriJointStateSyncPolicy>(
      CuriJointStateSyncPolicy(10), *curi_head_sub_, *panda_left_sub_, *panda_left_finger_sub_, *panda_right_sub_,
      *panda_right_finger_sub_, *curi_torso_sub_);
  joint_states_sync_->registerCallback(boost::bind(&MsgAggregator::curiJointStatesCB, this, _1, _2, _3, _4, _5, _6));
  curi_joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/curi/joint_states", 1);
  have_velocity_head_ = false;
  have_velocity_panda_left_ = false;
  have_velocity_panda_left_finger_ = false;
  have_velocity_panda_right_ = false;
  have_velocity_panda_right_finger_ = false;
  have_velocity_curi_torso_ = false;
  have_effort_head_ = false;
  have_effort_panda_left_ = false;
  have_effort_panda_left_finger_ = false;
  have_effort_panda_right_ = false;
  have_effort_panda_right_finger_ = false;
  have_effort_curi_torso_ = false;

  jointNames_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  name_prefix_head_ = "head_actuated_";
  name_prefix_panda_left_ = "panda_left_";
  name_prefix_panda_left_finger_ = "panda_left_finger_";
  name_prefix_panda_right_ = "panda_right_";
  name_prefix_panda_right_finger_ = "panda_left_finger_";
  name_prefix_torso_ = "torso_actuated_";
}

void MsgAggregator::curiJointStatesCB(const sensor_msgs::JointState::ConstPtr& curi_head,
                                      const sensor_msgs::JointState::ConstPtr& panda_left,
                                      const sensor_msgs::JointState::ConstPtr& panda_left_finger,
                                      const sensor_msgs::JointState::ConstPtr& panda_right,
                                      const sensor_msgs::JointState::ConstPtr& panda_right_finger,
                                      const sensor_msgs::JointState::ConstPtr& curi_torso) {
  sensor_msgs::JointState curi_current_joint_states;

  curi_current_joint_states.position.resize(curi_head->position.size() + panda_left->position.size() +
                                            panda_left_finger->position.size() + panda_right->position.size() +
                                            panda_right_finger->position.size() + curi_torso->position.size());
  curi_current_joint_states.velocity.resize(curi_head->position.size() + panda_left->position.size() +
                                            panda_left_finger->position.size() + panda_right->position.size() +
                                            panda_right_finger->position.size() + curi_torso->position.size());
  curi_current_joint_states.name.resize(curi_head->position.size() + panda_left->position.size() +
                                        panda_left_finger->position.size() + panda_right->position.size() +
                                        panda_right_finger->position.size() + curi_torso->position.size());
  curi_current_joint_states.effort.resize(curi_head->position.size() + panda_left->position.size() +
                                          panda_left_finger->position.size() + panda_right->position.size() +
                                          panda_right_finger->position.size() + curi_torso->position.size());

  curi_current_joint_states.header.frame_id = "frame_name";
  // Head
  for (int i = 0; i < curi_head->position.size(); ++i) {
    curi_current_joint_states.position[i] = curi_head->position[i];
    if (have_velocity_head_) {
      curi_current_joint_states.velocity[i] = curi_head->velocity[i];
    } else {
      curi_current_joint_states.velocity[i] = 0;
    }
    if (have_effort_head_) {
      curi_current_joint_states.effort[i] = curi_head->effort[i];
    } else {
      curi_current_joint_states.effort[i] = 0;
    }
    curi_current_joint_states.name[i] = name_prefix_head_ + jointNames_[i];
  }
  // Panda left
  for (int i = 0; i < panda_left->position.size(); ++i) {
    curi_current_joint_states.position[curi_head->position.size() + i] = panda_left->position[i];
    if (have_velocity_panda_left_) {
      curi_current_joint_states.velocity[curi_head->position.size() + i] = panda_left->velocity[i];
    } else {
      curi_current_joint_states.velocity[curi_head->position.size() + i] = 0;
    }
    if (have_effort_panda_left_) {
      curi_current_joint_states.effort[curi_head->position.size() + i] = panda_left->effort[i];
    } else {
      curi_current_joint_states.effort[curi_head->position.size() + i] = 0;
    }
    curi_current_joint_states.name[curi_head->position.size() + i] = name_prefix_panda_left_ + jointNames_[i];
  }
  // Panda Right
  for (int i = 0; i < panda_right->position.size(); ++i) {
    curi_current_joint_states.position[curi_head->position.size() + panda_left->position.size() + i] =
        panda_right->position[i];
    if (have_velocity_panda_right_) {
      curi_current_joint_states.velocity[curi_head->position.size() + panda_left->position.size() + i] =
          panda_right->velocity[i];
    } else {
      curi_current_joint_states.velocity[curi_head->position.size() + panda_left->position.size() + i] = 0;
    }
    if (have_effort_panda_right_) {
      curi_current_joint_states.effort[curi_head->position.size() + panda_left->position.size() + i] =
          panda_right->effort[i];
    } else {
      curi_current_joint_states.effort[curi_head->position.size() + panda_left->position.size() + i] = 0;
    }
    curi_current_joint_states.name[curi_head->position.size() + panda_left->position.size() + i] =
        name_prefix_panda_right_ + jointNames_[i];
  }
  // Panda Left Finger
  for (int i = 0; i < panda_left_finger->position.size(); ++i) {
    curi_current_joint_states
        .position[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() + i] =
        panda_left_finger->position[i];
    if (have_velocity_panda_left_finger_) {
      curi_current_joint_states
          .velocity[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() + i] =
          panda_left_finger->velocity[i];
    } else {
      curi_current_joint_states
          .velocity[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() + i] = 0;
    }
    if (have_effort_panda_left_finger_) {
      curi_current_joint_states
          .effort[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() + i] =
          panda_left_finger->effort[i];
    } else {
      curi_current_joint_states
          .effort[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() + i] = 0;
    }
    curi_current_joint_states
        .name[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() + i] =
        name_prefix_panda_left_finger_ + jointNames_[i];
  }
  // Panda Right Finger
  for (int i = 0; i < panda_right_finger->position.size(); ++i) {
    curi_current_joint_states.position[curi_head->position.size() + panda_left->position.size() +
                                       panda_right->position.size() + panda_left_finger->position.size() + i] =
        panda_right_finger->position[i];
    if (have_velocity_panda_right_finger_) {
      curi_current_joint_states.velocity[curi_head->position.size() + panda_left->position.size() +
                                         panda_right->position.size() + panda_left_finger->position.size() + i] =
          panda_right_finger->velocity[i];
    } else {
      curi_current_joint_states.velocity[curi_head->position.size() + panda_left->position.size() +
                                         panda_right->position.size() + panda_left_finger->position.size() + i] = 0;
    }
    if (have_effort_panda_right_finger_) {
      curi_current_joint_states.effort[curi_head->position.size() + panda_left->position.size() +
                                       panda_right->position.size() + panda_left_finger->position.size() + i] =
          panda_right_finger->effort[i];
    } else {
      curi_current_joint_states.effort[curi_head->position.size() + panda_left->position.size() +
                                       panda_right->position.size() + panda_left_finger->position.size() + i] = 0;
    }
    curi_current_joint_states.name[curi_head->position.size() + panda_left->position.size() +
                                   panda_right->position.size() + panda_left_finger->position.size() + i] =
        name_prefix_panda_right_finger_ + jointNames_[i];
  }
  // Panda Torso
  for (int i = 0; i < curi_torso->position.size(); ++i) {
    curi_current_joint_states
        .position[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() +
                  panda_left_finger->position.size() + panda_right_finger->position.size() + i] =
        curi_torso->position[i];
    if (have_velocity_curi_torso_) {
      curi_current_joint_states
          .velocity[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() +
                    panda_left_finger->position.size() + panda_right_finger->position.size() + i] =
          curi_torso->velocity[i];
    } else {
      curi_current_joint_states
          .velocity[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() +
                    panda_left_finger->position.size() + panda_right_finger->position.size() + i] = 0;
    }
    if (have_effort_curi_torso_) {
      curi_current_joint_states
          .effort[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() +
                  panda_left_finger->position.size() + panda_right_finger->position.size() + i] = curi_torso->effort[i];
    } else {
      curi_current_joint_states
          .effort[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() +
                  panda_left_finger->position.size() + panda_right_finger->position.size() + i] = 0;
    }
    curi_current_joint_states
        .name[curi_head->position.size() + panda_left->position.size() + panda_right->position.size() +
              panda_left_finger->position.size() + panda_right_finger->position.size() + i] =
        name_prefix_torso_ + jointNames_[i];
  }
  // Publish
  curi_joint_states_pub_.publish(curi_current_joint_states);
}
}  // namespace roport
