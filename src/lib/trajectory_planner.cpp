#include "roport/trajectory_planner.h"

namespace roport {

TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), visualize_(false), is_execute_(true), plant_(0.0), parser_(&plant_) {
  // Get all available planning group names
  XmlRpc::XmlRpcValue group_names;
  roport::getParam(nh_, pnh_, "group_names", group_names);
  ROS_ASSERT(group_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

  if (group_names.size() == 0) {
    throw std::runtime_error("Param group_names not defined");
  }

  // Create clients for getting the current group pose and executing pose command
  XmlRpc::XmlRpcValue get_group_pose_service_id;
  roport::getParam(nh_, pnh_, "get_group_pose_service_id", get_group_pose_service_id);
  ROS_ASSERT(get_group_pose_service_id.size() == group_names.size());

  XmlRpc::XmlRpcValue execute_group_cartesian_trajectory_service_id;
  roport::getParam(nh_, pnh_, "execute_group_cartesian_trajectory_service_id",
                   execute_group_cartesian_trajectory_service_id);
  ROS_ASSERT(execute_group_cartesian_trajectory_service_id.size() == group_names.size());

  for (int i = 0; i < group_names.size(); i++) {
    ROS_ASSERT(group_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string name = std::string(group_names[i]);

    std::string service_id = std::string(get_group_pose_service_id[i]);
    auto get_pose_client = nh_.serviceClient<roport::GetGroupPose>(service_id);
    if (!ros::service::waitForService(service_id, wait_for_service_timeout_)) {
      throw std::runtime_error("Failed to connect to get_group_pose service");
    }
    ROS_INFO("Created client for group '%s' to service %s", name.c_str(), service_id.c_str());

    service_id = std::string(execute_group_cartesian_trajectory_service_id[i]);
    auto execute_trajectory_client = nh_.serviceClient<roport::ExecuteGroupCartesianTrajectory>(service_id);
    if (!ros::service::waitForService(service_id, wait_for_service_timeout_)) {
      ROS_WARN_STREAM("Failed to connect to service: " << service_id);
      is_execute_ = false;
    } else {
      ROS_INFO("Created client for group '%s' to service %s", name.c_str(), service_id.c_str());
    }

    group_names_.push_back(name);
    get_current_pose_clients_.push_back(get_pose_client);
    execute_group_cartesian_trajectory_clients_.push_back(execute_trajectory_client);
  }

  // Get visualize flag
  XmlRpc::XmlRpcValue display_trajectory;
  roport::getParam(nh_, pnh_, "display_trajectory", display_trajectory);
  visualize_ = bool(display_trajectory);

  // Get joint names for each planning group
  XmlRpc::XmlRpcValue joint_names;
  roport::getParam(nh_, pnh_, "joint_names", joint_names);
  if (joint_names.size() != group_names.size()) {
    throw std::runtime_error("Param joint_names is not properly defined");
  }
  for (int i = 0; i < group_names_.size(); ++i) {
    ROS_ASSERT(joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<std::string> joint_name_list;
    for (int j = 0; j < joint_names[i].size(); ++j) {
      joint_name_list.push_back(joint_names[i][j]);
    }
    group_joint_names_.push_back(joint_name_list);
  }

  // Drake initialization
  XmlRpc::XmlRpcValue urdf;
  roport::getParam(nh_, pnh_, "robot_description", urdf);
  model_indexes_ = parser_.AddModelsFromString(std::string(urdf), "URDF");
  plant_.Finalize();

  // ROS interface initialization
  joint_state_subscriber_ =
      nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &TrajectoryPlanner::jointStatesCb, this);

  execute_all_cartesian_trajectory_srv_ = nh_.advertiseService(
      "execute_all_cartesian_trajectories", &TrajectoryPlanner::executeAllCartesianTrajectoriesCb, this);

  execute_joint_trajectory_with_cartesian_trajectories_srv_ =
      nh_.advertiseService("execute_joint_trajectory_with_cartesian_trajectories",
                           &TrajectoryPlanner::executeJointTrajectoryWithCartesianTrajectoriesCb, this);

  if (visualize_) {
    joint_trajectory_publisher_ =
        nh_.advertise<moveit_msgs::DisplayTrajectory>("trajectory_planner/joint_trajectory", 1);
    ROS_INFO("Planned joint trajectory can be visualized in RViz with:");
    ROS_INFO_STREAM("trajectory_planner/joint_trajectory");

    ROS_INFO("Planned cartesian trajectory can be visualized in RViz with:");
    for (const auto& name : group_names_) {
      std::string topic = "trajectory_planner/" + name + "/cartesian_trajectory";
      auto c_publisher = nh_.advertise<geometry_msgs::PoseArray>(topic, 1);
      cartesian_trajectory_publishers_.push_back(c_publisher);
      ROS_INFO_STREAM(topic);
    }
  }
}

void TrajectoryPlanner::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_ASSERT(current_joint_state_.name.size() == current_joint_state_.position.size());
  current_joint_state_ = *msg;
}

auto TrajectoryPlanner::executeAllCartesianTrajectoriesCb(roport::ExecuteAllCartesianTrajectories::Request& req,
                                                          roport::ExecuteAllCartesianTrajectories::Response& resp)
    -> bool {
  if (req.group_names.empty()) {
    ROS_WARN("The 'group_names' in the request is empty.");
    resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
    resp.result_msg = "Empty group_names";
    return true;
  }

  std::map<int, roport::CartesianTrajectory> trajectory_handler;
  for (size_t i = 0; i < req.group_names.size(); ++i) {
    auto result = roport::findInVector(group_names_, req.group_names[i]);
    if (!result.first) {
      ROS_WARN("Group name %s is not in known names, its trajectory will not be executed", req.group_names[i].c_str());
      resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
      resp.result_msg = "Invalid group name";
      return true;
    }

    auto index = result.second;
    auto reference_frame = req.trajectories[i].ref_frame;
    auto ee_frame = req.trajectories[i].ee_frame;

    roport::GetGroupPose get_current_pose;
    get_current_pose.request.group_name = group_names_[index];
    get_current_pose.request.ref_frame = reference_frame;
    get_current_pose.request.ee_frame = ee_frame;
    // Use the current pose as the initial commanded pose of the generated cartesian trajectory.
    get_current_pose_clients_[index].call(get_current_pose);

    if (get_current_pose.response.result_status == roport::GetGroupPoseResponse::SUCCEEDED) {
      roport::CartesianTrajectory cartesian_trajectory;
      TrajectoryPlanner::makeCartesianTrajectoryWithDrake(get_current_pose.response.pose, req.trajectories[i],
                                                          cartesian_trajectory);
      if (visualize_) {
        displayCartesianTrajectoryInRViz(index, cartesian_trajectory);
      }
      trajectory_handler.insert({index, cartesian_trajectory});
    } else {
      ROS_WARN("Fail to get the current pose for group %s", group_names_[index].c_str());
      resp.result_status = roport::ExecuteAllCartesianTrajectoriesResponse::FAILED;
      resp.result_msg = "Get current pose failed";
      return false;
    }
  }

  /** This naive implementation only allows executing one trajectory for one group for testing
  for (const auto& pair : trajectory_handler) {
    roport::ExecuteGroupPose execute_group_pose;
    execute_group_pose.request.ref_frame = pair.second.ref_frame;
    execute_group_pose.request.ee_frame = pair.second.ee_frame;
    execute_group_pose.request.group_name = group_names_[pair.first];
    for (int i = 0; i < pair.second.points.size(); i += 100) {
      auto p = pair.second.points[i];
      execute_group_pose.request.goal = p.pose;
      execute_group_pose.request.duration = p.duration * 100;
      execute_group_pose_clients_[pair.first].call(execute_group_pose);
      if (execute_group_pose.response.result_status != roport::ExecuteGroupPose::Response::SUCCEEDED) {
        ROS_ERROR("Call execute_group_pose service failed");
        break;
      }
    }
  }
  **/
  if (is_execute_) {
    // TODO use execute_group_cartesian_trajectory_clients_ to send dense trajectories to each group
  }
  resp.result_status = roport::ExecuteAllCartesianTrajectoriesResponse::SUCCEEDED;
  return true;
}

auto TrajectoryPlanner::executeJointTrajectoryWithCartesianTrajectoriesCb(
    roport::ExecuteAllCartesianTrajectories::Request& req,
    roport::ExecuteAllCartesianTrajectories::Response& resp) -> bool {
  if (req.group_names.empty()) {
    ROS_WARN("The 'group_names' in the execute all cartesian trajectory request is empty.");
    resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::FAILED;
    resp.result_msg = "Empty group_names";
    return false;
  }

  trajectory_msgs::JointTrajectory joint_trajectory;
  TrajectoryPlanner::makeJointTrajectoryWithDrake(req, joint_trajectory);
  if (visualize_) {
    displayJointTrajectoryInRViz(joint_trajectory);
  }

  resp.result_status = roport::ExecuteAllCartesianTrajectories::Response::SUCCEEDED;
  return true;
}

void TrajectoryPlanner::geometryPoseToRigidTransform(const geometry_msgs::Pose& p, drake::math::RigidTransformd& t) {
  Eigen::Matrix4d m;
  geometryPoseToEigenMatrix(p, m);
  t = drake::math::RigidTransformd(m);
}

void TrajectoryPlanner::rigidTransformToGeometryPose(const drake::math::RigidTransformd& t, geometry_msgs::Pose& p) {
  Eigen::Matrix4d m = t.GetAsMatrix4();
  eigenMatrixToGeometryPose(m, p);
}

void TrajectoryPlanner::currentJointStatesToDrakePosition(Eigen::VectorXd& q) {
  q = plant_.GetPositions(*plant_.CreateDefaultContext(), model_indexes_[0]);
  for (int i = 0; i < current_joint_state_.name.size(); ++i) {
    auto joint_name = current_joint_state_.name[i];
    auto joint_position = current_joint_state_.position[i];
    const auto& joint = plant_.GetJointByName(joint_name, model_indexes_[0]);
    int joint_index = joint.position_start();
    q[joint_index] = joint_position;
  }
}

void TrajectoryPlanner::jointTrajectoryPointToDrakePosition(const std::vector<std::string>& joint_names,
                                                            const trajectory_msgs::JointTrajectoryPoint& wp,
                                                            Eigen::VectorXd& q) {
  q = plant_.GetPositions(*plant_.CreateDefaultContext(), model_indexes_[0]);
  for (int i = 0; i < joint_names.size(); ++i) {
    const auto& name = joint_names[i];
    const auto& joint = plant_.GetJointByName(name, model_indexes_[0]);
    int joint_index = joint.position_start();
    q[joint_index] = wp.positions[i];
  }
}

void TrajectoryPlanner::drakePositionToJointTrajectoryPoint(const Eigen::VectorXd& q,
                                                            const double& time_from_start,
                                                            trajectory_msgs::JointTrajectoryPoint& wp) {
  ROS_ASSERT(q.size() == plant_.num_positions());
  // Select joint values to be added into trajectory by name
  std::vector<double> selected_q;
  for (const auto& joint_name : current_joint_state_.name) {
    const auto& joint = plant_.GetJointByName(joint_name, model_indexes_[0]);
    int joint_index = joint.position_start();
    selected_q.push_back(q[joint_index]);
  }
  wp.positions = selected_q;
  wp.time_from_start = ros::Duration(time_from_start);
}

void TrajectoryPlanner::rigidTransformToCartesianTrajectoryPoint(const drake::math::RigidTransformd& pose,
                                                                 const Eigen::Matrix<double, 6, 1>& vel,
                                                                 const Eigen::Matrix<double, 6, 1>& acc,
                                                                 const double& duration,
                                                                 roport::CartesianTrajectoryPoint& p) {
  rigidTransformToGeometryPose(pose, p.pose);
  eigenMatrixToGeometryTwist(vel, p.twist);
  eigenMatrixToGeometryAccel(acc, p.acceleration);
  p.duration = duration;
}

void TrajectoryPlanner::drakeTrajectoryToCartesianTrajectory(
    const drake::trajectories::PiecewisePose<double>& drake_trajectory,
    roport::CartesianTrajectory& trajectory) const {
  int trajectory_length = static_cast<int>(drake_trajectory.end_time() / default_time_step_);
  for (int i = 0; i <= trajectory_length; ++i) {
    auto time_stamp = default_time_step_ * i;
    auto pose = drake_trajectory.GetPose(time_stamp);
    auto vel = drake_trajectory.GetVelocity(time_stamp);
    auto acc = drake_trajectory.GetAcceleration(time_stamp);
    roport::CartesianTrajectoryPoint point;
    if (i == 0) {
      rigidTransformToCartesianTrajectoryPoint(pose, vel, acc, 0.0, point);
      logROSPose(point.pose);
    } else {
      rigidTransformToCartesianTrajectoryPoint(pose, vel, acc, default_time_step_, point);
      if (i == trajectory_length) {
        logROSPose(point.pose);
      }
    }
    trajectory.points.push_back(point);
  }
}

bool TrajectoryPlanner::makeCartesianTrajectoryWithDrake(geometry_msgs::Pose initial_pose,
                                                         const roport::CartesianTrajectory& sparse_trajectory,
                                                         roport::CartesianTrajectory& dense_trajectory) {
  dense_trajectory.header = sparse_trajectory.header;
  dense_trajectory.ref_frame = sparse_trajectory.ref_frame;
  dense_trajectory.ee_frame = sparse_trajectory.ee_frame;

  std::vector<double> times;
  std::vector<drake::math::RigidTransformd> poses;

  // The times for Drake's PiecewisePose record time_from_start, successive stamps must satisfy:
  // breaks_[i] - breaks_[i - 1] >= kEpsilonTime (i.e., std::numeric_limits<T>::epsilon = 2.22045e-16)
  double time_from_start = 0.0;
  times.push_back(time_from_start);  // The time_stamp for the initial_pose
  drake::math::RigidTransformd initial_t;
  geometryPoseToRigidTransform(initial_pose, initial_t);
  poses.push_back(initial_t);

  for (const auto& p : sparse_trajectory.points) {
    time_from_start += p.duration;
    times.push_back(time_from_start);
    drake::math::RigidTransformd intermediate_t;
    geometryPoseToRigidTransform(p.pose, intermediate_t);
    poses.push_back(intermediate_t);
  }

  auto drake_trajectory = drake::trajectories::PiecewisePose<double>::MakeLinear(times, poses);
  drakeTrajectoryToCartesianTrajectory(drake_trajectory, dense_trajectory);
  return true;
}

bool TrajectoryPlanner::makeJointTrajectoryWithDrake(const roport::ExecuteAllCartesianTrajectories::Request& request,
                                                     trajectory_msgs::JointTrajectory& joint_trajectory) {
  trajectory_msgs::JointTrajectory sparse_j_t;
  sparse_j_t.header = request.header;
  sparse_j_t.joint_names = current_joint_state_.name;
  if (generateJointTrajectoryWithIK(request.trajectories, sparse_j_t)) {
    generateDenseJointTrajectory(sparse_j_t, joint_trajectory);
    return true;
  }
  return false;
}

bool TrajectoryPlanner::generateJointTrajectoryWithIK(const std::vector<roport::CartesianTrajectory>& c_trajectories,
                                                      trajectory_msgs::JointTrajectory& sparse_j_trajectory) {
  auto lower_limits = plant_.GetPositionLowerLimits();
  auto upper_limits = plant_.GetPositionUpperLimits();

  // TODO release this limit:
  // Currently we assume the trajectory points of different groups share the same series of time stamps
  for (int i = 0; i < c_trajectories.size(); ++i) {
    ROS_ASSERT(c_trajectories[i].points.size() == c_trajectories[0].points.size());
  }

  double time_from_start = 0.0;
  // If time_from_start is not start from 0, add the current state as the initial state
  if (c_trajectories[0].points[0].duration > 0) {
    trajectory_msgs::JointTrajectoryPoint jtp;
    jtp.positions = current_joint_state_.position;
    jtp.velocities = current_joint_state_.velocity;
    jtp.time_from_start = ros::Duration(0);
    sparse_j_trajectory.points.push_back(jtp);
  }

  for (int j = 0; j < c_trajectories[0].points.size(); ++j) {
    drake::multibody::InverseKinematics ik(plant_);
    auto prog = ik.get_mutable_prog();
    for (int i = 0; i < c_trajectories.size(); ++i) {
      std::string pure_ee_frame;
      getSubStr(c_trajectories[i].ee_frame, '/', pure_ee_frame);
      std::string pure_ref_frame;
      getSubStr(c_trajectories[i].ref_frame, '/', pure_ref_frame);

      const auto& ee_frame = plant_.GetFrameByName(pure_ee_frame);
      const auto& ref_frame = plant_.GetFrameByName(pure_ref_frame);

      auto pose = c_trajectories[i].points[j].pose;
      drake::math::RigidTransformd trans;
      geometryPoseToRigidTransform(pose, trans);

      // Add pose constraint
      ik.AddPositionConstraint(ee_frame, drake::Vector3<double>::Zero(), ref_frame, trans.translation(),
                               trans.translation());
      ik.AddOrientationConstraint(ee_frame, drake::math::RotationMatrixd::Identity(), ref_frame, trans.rotation(), 0.0);
      // Add joint position constraint
      prog->AddBoundingBoxConstraint(lower_limits, upper_limits, ik.q());
    }
    Eigen::VectorXd initial_q;
    currentJointStatesToDrakePosition(initial_q);

    const auto& result = drake::solvers::Solve(*prog, initial_q);
    if (result.is_success()) {
      // solution type: Eigen::VectorXd is for all joints of the robot
      auto solution = result.GetSolution(ik.q());

      trajectory_msgs::JointTrajectoryPoint wp;
      time_from_start += c_trajectories[0].points[j].duration;
      drakePositionToJointTrajectoryPoint(solution, time_from_start, wp);

      sparse_j_trajectory.points.push_back(wp);
    } else {
      ROS_ERROR("IK solution failed for %i-th waypoints", j);
      return false;
    }
  }
  return true;
}

bool TrajectoryPlanner::generateDenseJointTrajectoryWithOptimization(
    const trajectory_msgs::JointTrajectory& sparse_joint_trajectory,
    trajectory_msgs::JointTrajectory& dense_joint_trajectory) {
  dense_joint_trajectory.header = sparse_joint_trajectory.header;
  dense_joint_trajectory.joint_names = sparse_joint_trajectory.joint_names;

  std::vector<double> times_from_start;
  std::vector<Eigen::MatrixXd> knots;
  for (const auto& waypoint : sparse_joint_trajectory.points) {
    times_from_start.push_back(waypoint.time_from_start.toSec());
    Eigen::VectorXd q;
    jointTrajectoryPointToDrakePosition(dense_joint_trajectory.joint_names, waypoint, q);
    knots.emplace_back(q);
  }
  auto trajectory =
      drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(times_from_start, knots);
  ROS_INFO_STREAM("nofs" << trajectory.get_number_of_segments());

  drake::solvers::MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(plant_.num_positions(), "q");
  auto v = prog.NewContinuousVariables(plant_.num_velocities(), "v");

  Eigen::VectorXd max_joint_vel = Eigen::VectorXd::Ones(plant_.num_velocities()) * 10;
  prog.AddConstraint(-max_joint_vel <= v);
  prog.AddConstraint(v <= max_joint_vel);

  for (size_t i = 0; i < times_from_start.size(); ++i) {
    Eigen::VectorXd desired_q = trajectory.value(times_from_start[i]);
    ROS_INFO_STREAM("Desired position at time" << times_from_start[i] << ":\n" << desired_q);
    prog.AddQuadraticErrorCost(Eigen::MatrixXd::Identity(plant_.num_positions(), plant_.num_positions()), desired_q, q);
  }

  Eigen::VectorXd initial_q;
  currentJointStatesToDrakePosition(initial_q);

  Eigen::VectorXd initial_dq = plant_.GetVelocities(*plant_.CreateDefaultContext(), model_indexes_[0]);
  Eigen::VectorXd initial_state(initial_q.size() + initial_dq.size());
  initial_state << initial_q, initial_dq;

  ROS_INFO_STREAM("Initial state:\n" << initial_state);
  auto result = drake::solvers::Solve(prog);
  if (result.is_success()) {
    const auto& solution = result.GetSolution();
    ROS_INFO_STREAM("Found solution:\n" << solution);
  } else {
    return false;
  }
  return true;
}

void TrajectoryPlanner::generateDenseJointTrajectory(const trajectory_msgs::JointTrajectory& sparse_joint_trajectory,
                                                     trajectory_msgs::JointTrajectory& dense_joint_trajectory) {
  dense_joint_trajectory.header = sparse_joint_trajectory.header;
  dense_joint_trajectory.joint_names = sparse_joint_trajectory.joint_names;

  std::vector<double> times_from_start;
  std::vector<Eigen::MatrixXd> knots;
  for (const auto& p : sparse_joint_trajectory.points) {
    times_from_start.push_back(p.time_from_start.toSec());
    Eigen::VectorXd q;
    jointTrajectoryPointToDrakePosition(dense_joint_trajectory.joint_names, p, q);
    knots.emplace_back(q);
  }
  auto trajectory =
      drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(times_from_start, knots);

  for (double t = trajectory.start_time(); t <= trajectory.end_time(); t += joint_traj_time_step_) {
    Eigen::VectorXd q = trajectory.value(t);
    trajectory_msgs::JointTrajectoryPoint wp;
    drakePositionToJointTrajectoryPoint(q, t, wp);
    dense_joint_trajectory.points.push_back(wp);
  }
}

bool TrajectoryPlanner::optimizeDrakeJointTrajectory(const drake::trajectories::PiecewisePolynomial<double>& traj,
                                                     drake::trajectories::PiecewisePolynomial<double>& traj_opt) {
  // Exception thrown while processing service call: The specified input port is abstract-valued,
  // and this constraint only supports vector-valued input ports.
  // Did you perhaps forget to pass a non-default `input_port_index` argument?
  const auto& input_port = plant_.get_actuation_input_port();
  const auto& input_port_index = input_port.get_index();
  auto optimizer = DirectCollocation(&plant_, *plant_.CreateDefaultContext(), 12, joint_traj_time_step_,
                                     joint_traj_time_step_, input_port_index);

  const double kMaxSpeed = 1000;  //  rad/s
  optimizer.AddEqualTimeIntervalsConstraints();

  Eigen::VectorXd initial_states = Eigen::VectorXd::Zero(plant_.num_positions() + plant_.num_velocities());
  initial_states.segment(0, plant_.num_positions()) = traj.value(0);

  optimizer.AddConstraintToAllKnotPoints(optimizer.initial_state() == initial_states);
  auto s = optimizer.state();
  auto u = optimizer.input();
  for (int k = 0; k < plant_.num_velocities(); ++k) {
    optimizer.AddConstraintToAllKnotPoints(optimizer.state()(k + plant_.num_positions()) <= kMaxSpeed);
    optimizer.AddConstraintToAllKnotPoints(optimizer.state()(k + plant_.num_positions()) >= -kMaxSpeed);
  }

  optimizer.SetInitialTrajectory(drake::trajectories::PiecewisePolynomial<double>(), traj);

  auto traj_result = drake::solvers::Solve(optimizer.prog());
  if (traj_result.is_success()) {
    ROS_INFO_STREAM("Trajectory optimized");
    traj_opt = optimizer.ReconstructStateTrajectory(traj_result);
    ROS_INFO_STREAM(traj_opt.value(3));
    return true;
  }
  ROS_WARN_STREAM("Failed to optimize trajectory");
  return false;
}

bool TrajectoryPlanner::optimizePiecewisePolynomial(const drake::trajectories::PiecewisePolynomial<double>& traj,
                                                    drake::trajectories::PiecewisePolynomial<double>& traj_opt) {
  //  drake::solvers::MathematicalProgram prog;
  //  auto q = prog.NewContinuousVariables(plant_.num_positions(), "q");
  //  auto v = prog.NewContinuousVariables(plant_.num_velocities(), "v");
  //
  //  Eigen::VectorXd max_joint_vel = Eigen::VectorXd::Ones(plant_.num_velocities()) * 10;
  //  prog.AddConstraint(-max_joint_vel <= v);
  //  prog.AddConstraint(v <= max_joint_vel);
  //
  //  for (size_t i = 0; i < times_from_start.size(); ++i) {
  //    Eigen::VectorXd desired_q = traj.value(times_from_start[i]);
  //    ROS_INFO_STREAM("Desired position at time" << times_from_start[i] << ":\n" << desired_q);
  //    prog.AddQuadraticErrorCost(Eigen::MatrixXd::Identity(plant_.num_positions(), plant_.num_positions()), desired_q,
  //    q);
  //  }
  //
  //  Eigen::VectorXd initial_q;
  //  currentJointStatesToDrakePosition(initial_q);
  //
  //  Eigen::VectorXd initial_dq = plant_.GetVelocities(*plant_.CreateDefaultContext(), model_indexes_[0]);
  //  Eigen::VectorXd initial_state(initial_q.size() + initial_dq.size());
  //  initial_state << initial_q, initial_dq;
  //
  //  ROS_INFO_STREAM("Initial state:\n" << initial_state);
  //  auto result = drake::solvers::Solve(prog);
  //  if (result.is_success()) {
  //    const auto& solution = result.GetSolution(q);
  //    ROS_INFO_STREAM("Found solution:\n" << solution);
  //  } else {
  //    return false;
  //  }
  return true;
}

bool TrajectoryPlanner::makeJointTrajectoryWithDrake(const roport::CartesianTrajectory& sparse_trajectory,
                                                     trajectory_msgs::JointTrajectory& joint_trajectory) {
  joint_trajectory.header = sparse_trajectory.header;
  joint_trajectory.joint_names = current_joint_state_.name;

  std::string pure_ee_frame;
  getSubStr(sparse_trajectory.ee_frame, '/', pure_ee_frame);
  std::string pure_ref_frame;
  getSubStr(sparse_trajectory.ref_frame, '/', pure_ref_frame);

  const auto& ee_frame = plant_.GetFrameByName(pure_ee_frame);
  const auto& ref_frame = plant_.GetFrameByName(pure_ref_frame);

  auto lower_limits = plant_.GetPositionLowerLimits();
  auto upper_limits = plant_.GetPositionUpperLimits();

  double time_from_start = 0.0;
  for (const auto& point : sparse_trajectory.points) {
    drake::multibody::InverseKinematics ik(plant_);
    drake::math::RigidTransformd t;
    geometryPoseToRigidTransform(point.pose, t);

    // Add pose constraint
    ik.AddPositionConstraint(ee_frame, drake::Vector3<double>::Zero(), ref_frame, t.translation(), t.translation());
    ik.AddOrientationConstraint(ee_frame, drake::math::RotationMatrixd::Identity(), ref_frame, t.rotation(), 0.0);
    // Add joint position constraint
    auto prog = ik.get_mutable_prog();
    prog->AddBoundingBoxConstraint(lower_limits, upper_limits, ik.q());

    // For floating base robots, the first 7 vars in initial_state represents the pose of the base frame
    // wrt the world frame, which could be 1 0 0 0 0 0 0, i.e., the first 4 vars represent orientation,
    // and the last 3 represent translation.
    Eigen::VectorXd initial_q;
    currentJointStatesToDrakePosition(initial_q);

    const auto& result = drake::solvers::Solve(*prog, initial_q);
    if (result.is_success()) {
      // solution type: Eigen::VectorXd is for all joints of the robot
      auto solution = result.GetSolution(ik.q());

      auto traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold({0.0, 1.0}, {initial_q, solution});

      // Exception thrown while processing service call: The specified input port is abstract-valued,
      // and this constraint only supports vector-valued input ports.
      // Did you perhaps forget to pass a non-default `input_port_index` argument?
      const auto& input_port = plant_.get_actuation_input_port();
      const auto& input_port_index = input_port.get_index();
      auto traj_opt = drake::planning::trajectory_optimization::DirectCollocation(
          &plant_, *plant_.CreateDefaultContext(), 10, 0.1, 0.1, input_port_index);

      const double kMaxSpeed = 0.1;  //  rad/s
      traj_opt.AddEqualTimeIntervalsConstraints();
      ROS_INFO_STREAM(traj_opt.initial_state());
      Eigen::VectorXd initial_states = Eigen::VectorXd::Zero(plant_.num_positions() + plant_.num_velocities());
      initial_states.segment(0, plant_.num_positions()) = initial_q;
      traj_opt.AddConstraintToAllKnotPoints(traj_opt.initial_state() == initial_q);
      traj_opt.AddConstraintToAllKnotPoints(traj_opt.state()(7, plant_.num_positions()) <= kMaxSpeed);
      traj_opt.AddConstraintToAllKnotPoints(traj_opt.state()(7, plant_.num_positions()) >= -kMaxSpeed);

      auto traj_result = drake::solvers::Solve(traj_opt.prog());
      if (traj_result.is_success()) {
        auto optimized_traj = traj_opt.ReconstructInputTrajectory(traj_result);
        ROS_INFO_STREAM("Trajectory optimized " << optimized_traj.value(0));
      } else {
        ROS_WARN_STREAM("Failed to optimize trajectory");
      }

      // Select joint values to be added into trajectory by name
      std::vector<double> joint_positions(solution.data(), solution.data() + solution.size());
      std::vector<double> selected_joint_positions;
      for (const auto& joint_name : current_joint_state_.name) {
        const auto& joint = plant_.GetJointByName(joint_name, model_indexes_[0]);
        int joint_index = joint.position_start();
        auto joint_position = joint_positions[joint_index];
        selected_joint_positions.push_back(joint_position);
      }

      trajectory_msgs::JointTrajectoryPoint j_point;
      j_point.positions = selected_joint_positions;

      time_from_start += point.duration;
      j_point.time_from_start = ros::Duration(time_from_start);
      joint_trajectory.points.push_back(j_point);
    } else {
      ROS_ERROR("IK solution failed for pose:");
      logROSPose(point.pose);
      return false;
    }
  }
  return true;
}

void TrajectoryPlanner::displayCartesianTrajectoryInRViz(const int& index,
                                                         const roport::CartesianTrajectory& cartesian_trajectory,
                                                         const int& step) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = cartesian_trajectory.ref_frame;
  for (int i = 0; i < cartesian_trajectory.points.size(); i += step) {
    pose_array.poses.push_back(cartesian_trajectory.points[i].pose);
  }
  cartesian_trajectory_publishers_[index].publish(pose_array);
}

void TrajectoryPlanner::displayJointTrajectoryInRViz(const trajectory_msgs::JointTrajectory& joint_trajectory) {
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.model_id = "";

  moveit_msgs::RobotState trajectory_start;
  trajectory_start.joint_state = current_joint_state_;
  display_trajectory.trajectory_start = trajectory_start;

  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  display_trajectory.trajectory.push_back(robot_trajectory);

  joint_trajectory_publisher_.publish(display_trajectory);
}
}  // namespace roport
