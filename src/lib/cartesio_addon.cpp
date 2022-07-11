#include <OpenSoT/tasks/velocity/CentauroAnkleSteering.h>
#include <OpenSoT/tasks/velocity/PureRolling.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>
#include <cartesian_interface/sdk/problem/Task.h>
#include <boost/make_shared.hpp>
#include <utility>

using namespace XBot::Cartesian;

struct CentauroSteeringTask : public TaskDescriptionImpl {
  CARTESIO_DECLARE_SMART_PTR(CentauroSteeringTask)

  std::string wheel_name;
  double max_steering_speed;
  Eigen::Vector3d contact_plane_normal;

  CentauroSteeringTask(YAML::Node task_node, Context::ConstPtr ctx);
};

CentauroSteeringTask::CentauroSteeringTask(YAML::Node task_node, Context::ConstPtr ctx)
    : TaskDescriptionImpl(task_node, std::move(ctx), "steering_" + task_node["wheel_name"].as<std::string>(), 1),
      contact_plane_normal(0, 0, 1.) {
  wheel_name = task_node["wheel_name"].as<std::string>();

  if (task_node["max_steering_speed"]) {
    max_steering_speed = task_node["max_steering_speed"].as<double>();
  } else {
    max_steering_speed = 3.0;
  }

  if (task_node["contact_plane_normal"]) {
    auto contact_plane_normal_std = task_node["contact_plane_normal"].as<std::vector<double>>();

    if (contact_plane_normal_std.size() != 3) {
      throw std::runtime_error("contact_plane_normal size() must be 3");
    }

    contact_plane_normal = Eigen::Vector3d::Map(contact_plane_normal_std.data());
  }
}

CARTESIO_REGISTER_TASK_PLUGIN(CentauroSteeringTask, CentauroSteering);

class CentauroSteeringOpenSot : public OpenSotTaskAdapter {
 public:
  CentauroSteeringOpenSot(const TaskDescription::Ptr& task_desc, Context::ConstPtr ctx)
      : OpenSotTaskAdapter(task_desc, std::move(ctx)) {
    ci_steering_ = std::dynamic_pointer_cast<CentauroSteeringTask>(task_desc);

    if (!ci_steering_)
      throw std::runtime_error(
          "Provided task description "
          "does not have expected type 'CentauroSteeringTask'");
  }

  TaskPtr constructTask() override {
    sot_steering_ = boost::make_shared<OpenSoT::tasks::velocity::CentauroAnkleSteering>(
        ci_steering_->wheel_name, _model, _ctx->params()->getControlPeriod(), ci_steering_->max_steering_speed);

    return sot_steering_;
  }

 private:
  boost::shared_ptr<OpenSoT::tasks::velocity::CentauroAnkleSteering> sot_steering_;
  CentauroSteeringTask::Ptr ci_steering_;
};

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(CentauroSteeringOpenSot, CentauroSteering);

struct WheelRollingTask : public TaskDescriptionImpl {
  CARTESIO_DECLARE_SMART_PTR(WheelRollingTask)

  std::string wheel_name;
  double wheel_radius;
  Eigen::Vector3d contact_plane_normal;
  bool include_z_axis;

  WheelRollingTask(YAML::Node task_node, Context::ConstPtr ctx);
};

WheelRollingTask::WheelRollingTask(YAML::Node task_node, Context::ConstPtr ctx)
    : TaskDescriptionImpl(task_node, std::move(ctx), "rolling_" + task_node["wheel_name"].as<std::string>(), 2),
      contact_plane_normal(0, 0, 1.),
      include_z_axis(false) {
  wheel_name = task_node["wheel_name"].as<std::string>();

  wheel_radius = task_node["wheel_radius"].as<double>();

  if (auto n = task_node["contact_plane_normal"]) {
    auto contact_plane_normal_std = n.as<std::vector<double>>();

    if (contact_plane_normal_std.size() != 3) {
      throw std::runtime_error("contact_plane_normal size() must be 3");
    }

    contact_plane_normal = Eigen::Vector3d::Map(contact_plane_normal_std.data());
  }

  if (auto n = task_node["include_z_axis"]) {
    include_z_axis = n.as<bool>();
  }
}

CARTESIO_REGISTER_TASK_PLUGIN(WheelRollingTask, WheelRolling)

class WheelRollingOpenSot : public OpenSotTaskAdapter {
 public:
  WheelRollingOpenSot(const TaskDescription::Ptr& task_desc, Context::ConstPtr ctx)
      : OpenSotTaskAdapter(task_desc, std::move(ctx)) {
    ci_rolling_ = std::dynamic_pointer_cast<WheelRollingTask>(task_desc);

    if (!ci_rolling_)
      throw std::runtime_error(
          "Provided task description "
          "does not have expected type 'WheelRollingTask'");
  }

  TaskPtr constructTask() override {
    auto sot_rolling = boost::make_shared<OpenSoT::tasks::velocity::PureRollingPosition>(
        ci_rolling_->wheel_name, ci_rolling_->wheel_radius, *_model, ci_rolling_->include_z_axis);

    return sot_rolling;
  }

 private:
  WheelRollingTask::Ptr ci_rolling_;
};

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(WheelRollingOpenSot, WheelRolling);
