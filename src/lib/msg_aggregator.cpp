//
// Created by Ray on 06/04/2022.
//

#include "roport/msg_aggregator.h"

namespace roport {

MsgAggregator::MsgAggregator(const ros::NodeHandle& node_handle, const ros::NodeHandle& pnh)
    : nh_(node_handle), pnh_(pnh), high_frequency_num_(0), low_frequency_num_(0) {
  init();  // NOLINT

  std::string target_js_topic = "/joint_state";
  if (!pnh_.param<std::string>("target_js_topic", target_js_topic, "/joint_state")) {
    if (!nh_.param<std::string>("target_js_topic", target_js_topic, "/joint_state")) {
      ROS_WARN_STREAM(prefix << "Param target_js_topic is not set, using default: " << target_js_topic);
    }
  }
  ROS_INFO_STREAM(prefix << "Publish to the target topic: " << target_js_topic);
  publisher_ = nh_.advertise<sensor_msgs::JointState>(target_js_topic, 1);
}

void MsgAggregator::init() {
  XmlRpc::XmlRpcValue high_frequency_topics;
  XmlRpc::XmlRpcValue high_frequency_names;
  getParam("high_frequency_topics", high_frequency_topics);
  getParam("high_frequency_names", high_frequency_names);
  if (high_frequency_topics.size() != high_frequency_names.size()) {  // NOLINT
    throw std::runtime_error("High frequency topics size does not match names size");
  }
  high_frequency_num_ = high_frequency_topics.size();
  if (high_frequency_num_ < 2 || high_frequency_num_ > 3) {  // NOLINT
    throw std::runtime_error("Only 2 or 3 high frequency topics are supported for now");
  }

  XmlRpc::XmlRpcValue low_frequency_topics;
  XmlRpc::XmlRpcValue low_frequency_names;
  getParam("low_frequency_topics", low_frequency_topics);
  getParam("low_frequency_names", low_frequency_names);
  if (low_frequency_topics.size() != low_frequency_names.size()) {  // NOLINT
    throw std::runtime_error("Low frequency topics size does not match names size");
  }
  low_frequency_num_ = low_frequency_topics.size();
  if (low_frequency_num_ < 2 || low_frequency_num_ > 3) {  // NOLINT
    throw std::runtime_error("Only 2 or 3 low frequency topics are supported for now");
  }

  // Handle low frequency topics
  for (int idx = 0; idx < low_frequency_num_; ++idx) {  // NOLINT
    XmlRpc::XmlRpcValue entity = low_frequency_topics[idx];

    XmlRpc::XmlRpcValue name_group = low_frequency_names[idx];
    std::vector<std::string> names;
    for (int jdx = 0; jdx < name_group.size(); ++jdx) {  // NOLINT
      names.emplace_back(name_group[jdx]);
    }
    ROS_INFO_STREAM(prefix << "Low frequency topic " << entity << " has " << name_group.size() << " names:\n"
                           << name_group);
    low_frequency_topics_.emplace_back(entity);
    low_frequency_name_groups_.push_back(names);

    auto subscriber = std::make_shared<MsgSubscriber>(nh_, std::string(entity), 1);
    low_frequency_subscribers_.push_back(subscriber);
  }

  size_t name_dim = 0;
  for (auto& name_group : low_frequency_name_groups_) {
    name_dim += name_group.size();
  }
  low_frequency_joint_state_.name.resize(name_dim);
  low_frequency_joint_state_.position.resize(name_dim);
  low_frequency_joint_state_.velocity.resize(name_dim);
  low_frequency_joint_state_.effort.resize(name_dim);

  if (low_frequency_num_ == 2) {
    duo_low_frequency_synchronizer_ = std::make_shared<DuoPolicySynchronizer>(
        DuoPolicy(10), *low_frequency_subscribers_[0], *low_frequency_subscribers_[1]);

    duo_low_frequency_synchronizer_->registerCallback(
        boost::bind(&MsgAggregator::lowFrequencyCB, this, _1, _2));  // NOLINT
  } else {
    tri_low_frequency_synchronizer_ = std::make_shared<TriPolicySynchronizer>(
        TriPolicy(10), *low_frequency_subscribers_[0], *low_frequency_subscribers_[1], *low_frequency_subscribers_[2]);

    tri_low_frequency_synchronizer_->registerCallback(
        boost::bind(&MsgAggregator::lowFrequencyCB, this, _1, _2, _3));  // NOLINT
  }

  // Handle high frequency topics
  for (int idx = 0; idx < high_frequency_num_; ++idx) {
    XmlRpc::XmlRpcValue entity = high_frequency_topics[idx];

    XmlRpc::XmlRpcValue name_group = high_frequency_names[idx];
    std::vector<std::string> names;
    for (int jdx = 0; jdx < name_group.size(); ++jdx) {  // NOLINT
      names.emplace_back(name_group[jdx]);
    }
    ROS_INFO_STREAM(prefix << "High frequency topic " << entity << " has " << name_group.size() << " names:\n"
                           << name_group);
    high_frequency_topics_.emplace_back(entity);
    high_frequency_name_groups_.push_back(names);

    auto subscriber = std::make_shared<MsgSubscriber>(nh_, std::string(entity), 1);
    high_frequency_subscribers_.push_back(subscriber);
  }
  if (high_frequency_num_ == 2) {
    duo_high_frequency_synchronizer_ = std::make_shared<DuoPolicySynchronizer>(
        DuoPolicy(10), *high_frequency_subscribers_[0], *high_frequency_subscribers_[1]);

    duo_high_frequency_synchronizer_->registerCallback(
        boost::bind(&MsgAggregator::highFrequencyCB, this, _1, _2));  // NOLINT
  } else {
    tri_high_frequency_synchronizer_ =
        std::make_shared<TriPolicySynchronizer>(TriPolicy(10), *high_frequency_subscribers_[0],
                                                *high_frequency_subscribers_[1], *high_frequency_subscribers_[2]);

    tri_high_frequency_synchronizer_->registerCallback(
        boost::bind(&MsgAggregator::highFrequencyCB, this, _1, _2, _3));  // NOLINT
  }
}

void MsgAggregator::highFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1,
                                    const sensor_msgs::JointState::ConstPtr& msg_2) {
  sensor_msgs::JointState joint_state;
  size_t name_dim = 0;
  std::vector<std::string> names;
  for (const auto& name_group : high_frequency_name_groups_) {
    name_dim += name_group.size();
    for (const auto& name : name_group) {
      names.push_back(name);
    }
  }
  size_t position_dim = msg_1->position.size() + msg_2->position.size();

  if (name_dim != position_dim) {
    ROS_ERROR_STREAM_ONCE(prefix << "High frequency names' size " << name_dim << " does not match " << " position size "
                                 << position_dim);
    return;
  }

  joint_state.name.resize(name_dim);
  joint_state.position.resize(name_dim);
  joint_state.velocity.resize(name_dim);
  joint_state.effort.resize(name_dim);

  std::copy(names.begin(), names.end(), joint_state.name.begin());

  std::copy(msg_1->position.begin(), msg_1->position.end(), joint_state.position.begin());
  if (msg_1->velocity.size() == msg_1->position.size()) {
    std::copy(msg_1->velocity.begin(), msg_1->velocity.end(), joint_state.velocity.begin());
  }
  if (msg_1->effort.size() == msg_1->position.size()) {
    std::copy(msg_1->effort.begin(), msg_1->effort.end(), joint_state.effort.begin());
  }

  long previous_size = msg_1->position.size();  // NOLINT
  std::copy(msg_2->position.begin(), msg_2->position.end(), joint_state.position.begin() + previous_size);
  if (msg_2->velocity.size() == msg_2->position.size()) {
    std::copy(msg_2->velocity.begin(), msg_2->velocity.end(), joint_state.velocity.begin() + previous_size);
  }
  if (msg_2->effort.size() == msg_2->position.size()) {
    std::copy(msg_2->effort.begin(), msg_2->effort.end(), joint_state.effort.begin() + previous_size);
  }

  publishCombined(joint_state);
}

void MsgAggregator::highFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1,
                                    const sensor_msgs::JointState::ConstPtr& msg_2,
                                    const sensor_msgs::JointState::ConstPtr& msg_3) {
  sensor_msgs::JointState joint_state;
  size_t name_dim = 0;
  std::vector<std::string> names;
  for (const auto& name_group : high_frequency_name_groups_) {
    name_dim += name_group.size();
    for (const auto& name : name_group) {
      names.push_back(name);
    }
  }
  size_t position_dim = msg_1->position.size() + msg_2->position.size() + msg_3->position.size();

  if (name_dim != position_dim) {
    ROS_ERROR_STREAM_ONCE(prefix << "High frequency names' size " << name_dim << " does not match " << " position size "
                                 << position_dim);
    return;
  }

  joint_state.name.resize(name_dim);
  joint_state.position.resize(name_dim);
  joint_state.velocity.resize(name_dim);
  joint_state.effort.resize(name_dim);

  std::copy(names.begin(), names.end(), joint_state.name.begin());

  std::copy(msg_1->position.begin(), msg_1->position.end(), joint_state.position.begin());
  if (msg_1->velocity.size() == msg_1->position.size()) {
    std::copy(msg_1->velocity.begin(), msg_1->velocity.end(), joint_state.velocity.begin());
  }
  if (msg_1->effort.size() == msg_1->position.size()) {
    std::copy(msg_1->effort.begin(), msg_1->effort.end(), joint_state.effort.begin());
  }

  long previous_size = msg_1->position.size();  // NOLINT
  std::copy(msg_2->position.begin(), msg_2->position.end(), joint_state.position.begin() + previous_size);
  if (msg_2->velocity.size() == msg_2->position.size()) {
    std::copy(msg_2->velocity.begin(), msg_2->velocity.end(), joint_state.velocity.begin() + previous_size);
  }
  if (msg_2->effort.size() == msg_2->position.size()) {
    std::copy(msg_2->effort.begin(), msg_2->effort.end(), joint_state.effort.begin() + previous_size);
  }

  previous_size = msg_1->position.size() + msg_2->position.size();  // NOLINT
  std::copy(msg_3->position.begin(), msg_3->position.end(), joint_state.position.begin() + previous_size);
  if (msg_3->velocity.size() == msg_3->position.size()) {
    std::copy(msg_3->velocity.begin(), msg_3->velocity.end(), joint_state.velocity.begin() + previous_size);
  }
  if (msg_3->effort.size() == msg_3->position.size()) {
    std::copy(msg_3->effort.begin(), msg_3->effort.end(), joint_state.effort.begin() + previous_size);
  }

  publishCombined(joint_state);
}

void MsgAggregator::lowFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1,
                                   const sensor_msgs::JointState::ConstPtr& msg_2) {
  size_t position_dim = msg_1->position.size() + msg_2->position.size();
  if (low_frequency_joint_state_.name.size() != position_dim) {
    ROS_ERROR_STREAM_ONCE(prefix << "Low frequency names' size " << low_frequency_joint_state_.name.size()
                                 << " does not match position size " << position_dim);
    return;
  }

  std::vector<std::string> names;
  for (const auto& name_group : low_frequency_name_groups_) {
    for (const auto& name : name_group) {
      names.push_back(name);
    }
  }

  std::copy(names.begin(), names.end(), low_frequency_joint_state_.name.begin());

  std::copy(msg_1->position.begin(), msg_1->position.end(), low_frequency_joint_state_.position.begin());
  if (msg_1->velocity.size() == msg_1->position.size()) {
    std::copy(msg_1->velocity.begin(), msg_1->velocity.end(), low_frequency_joint_state_.velocity.begin());
  }
  if (msg_1->effort.size() == msg_1->position.size()) {
    std::copy(msg_1->effort.begin(), msg_1->effort.end(), low_frequency_joint_state_.effort.begin());
  }

  long previous_size = msg_1->position.size();  // NOLINT
  std::copy(msg_2->position.begin(), msg_2->position.end(),
            low_frequency_joint_state_.position.begin() + previous_size);
  if (msg_2->velocity.size() == msg_2->position.size()) {
    std::copy(msg_2->velocity.begin(), msg_2->velocity.end(),
              low_frequency_joint_state_.velocity.begin() + previous_size);
  }
  if (msg_2->effort.size() == msg_2->position.size()) {
    std::copy(msg_2->effort.begin(), msg_2->effort.end(), low_frequency_joint_state_.effort.begin() + previous_size);
  }
}

void MsgAggregator::lowFrequencyCB(const sensor_msgs::JointState::ConstPtr& msg_1,
                                   const sensor_msgs::JointState::ConstPtr& msg_2,
                                   const sensor_msgs::JointState::ConstPtr& msg_3) {
  size_t position_dim = msg_1->position.size() + msg_2->position.size() + msg_3->position.size();
  if (low_frequency_joint_state_.name.size() != position_dim) {
    ROS_ERROR_STREAM_ONCE(prefix << "Low frequency names' size " << low_frequency_joint_state_.name.size()
                                 << " does not match position size " << position_dim);
    return;
  }

  std::vector<std::string> names;
  for (const auto& name_group : low_frequency_name_groups_) {
    for (const auto& name : name_group) {
      names.push_back(name);
    }
  }

  std::copy(names.begin(), names.end(), low_frequency_joint_state_.name.begin());

  std::copy(msg_1->position.begin(), msg_1->position.end(), low_frequency_joint_state_.position.begin());
  if (msg_1->velocity.size() == msg_1->position.size()) {
    std::copy(msg_1->velocity.begin(), msg_1->velocity.end(), low_frequency_joint_state_.velocity.begin());
  }
  if (msg_1->effort.size() == msg_1->position.size()) {
    std::copy(msg_1->effort.begin(), msg_1->effort.end(), low_frequency_joint_state_.effort.begin());
  }

  long previous_size = msg_1->position.size();  // NOLINT
  std::copy(msg_2->position.begin(), msg_2->position.end(),
            low_frequency_joint_state_.position.begin() + previous_size);
  if (msg_2->velocity.size() == msg_2->position.size()) {
    std::copy(msg_2->velocity.begin(), msg_2->velocity.end(),
              low_frequency_joint_state_.velocity.begin() + previous_size);
  }
  if (msg_2->effort.size() == msg_2->position.size()) {
    std::copy(msg_2->effort.begin(), msg_2->effort.end(), low_frequency_joint_state_.effort.begin() + previous_size);
  }

  previous_size = msg_1->position.size() + msg_2->position.size();  // NOLINT
  std::copy(msg_3->position.begin(), msg_3->position.end(),
            low_frequency_joint_state_.position.begin() + previous_size);
  if (msg_3->velocity.size() == msg_3->position.size()) {
    std::copy(msg_3->velocity.begin(), msg_3->velocity.end(),
              low_frequency_joint_state_.velocity.begin() + previous_size);
  }
  if (msg_3->effort.size() == msg_3->position.size()) {
    std::copy(msg_3->effort.begin(), msg_3->effort.end(), low_frequency_joint_state_.effort.begin() + previous_size);
  }
}

void MsgAggregator::publishCombined(const sensor_msgs::JointState& high_frequency_msg) {
  sensor_msgs::JointState combined_msg;
  auto low_dim = low_frequency_joint_state_.name.size();
  auto high_dim = high_frequency_msg.name.size();

  combined_msg.name.resize(low_dim + high_dim);
  combined_msg.position.resize(low_dim + high_dim);
  combined_msg.velocity.resize(low_dim + high_dim);
  combined_msg.effort.resize(low_dim + high_dim);

  std::copy(high_frequency_msg.name.begin(), high_frequency_msg.name.end(), combined_msg.name.begin());
  std::copy(high_frequency_msg.position.begin(), high_frequency_msg.position.end(), combined_msg.position.begin());
  std::copy(high_frequency_msg.velocity.begin(), high_frequency_msg.velocity.end(), combined_msg.velocity.begin());
  std::copy(high_frequency_msg.effort.begin(), high_frequency_msg.effort.end(), combined_msg.effort.begin());

  long previous_size = static_cast<long>(high_dim);
  std::copy(low_frequency_joint_state_.name.begin(), low_frequency_joint_state_.name.end(),
            combined_msg.name.begin() + previous_size);
  std::copy(low_frequency_joint_state_.position.begin(), low_frequency_joint_state_.position.end(),
            combined_msg.position.begin() + previous_size);
  std::copy(low_frequency_joint_state_.velocity.begin(), low_frequency_joint_state_.velocity.end(),
            combined_msg.velocity.begin() + previous_size);
  std::copy(low_frequency_joint_state_.effort.begin(), low_frequency_joint_state_.effort.end(),
            combined_msg.effort.begin() + previous_size);

  publisher_.publish(combined_msg);
  ROS_INFO_STREAM_ONCE("Publishing " << publisher_.getTopic() << " ... (print only once)");
}

}  // namespace roport
