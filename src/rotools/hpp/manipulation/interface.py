from __future__ import print_function

import math
import time

import numpy as np
from collections import namedtuple

import rospy

from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule, \
    Constraints, ConstraintGraphFactory, Client, SecurityMargins
from hpp.corbaserver import loadServerPlugin

from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory

from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from rotools.hpp.core import Object, Environment, Model, Gripper
from rotools.utility import common, transform


class HPPManipulationInterface(object):

    def __init__(
            self,
            env_name,
            env_pkg_name,
            env_surface,
            object_name,
            object_pkg_name,
            object_surface,
            object_handle,
            robot_name,
            robot_pkg_name,
            robot_urdf_name,
            robot_srdf_name,
            robot_bound,
            object_bound,
            gripper_name,
            fingers,
            finger_joints,
            finger_joint_values,
            joint_cmd_topic,
            base_cmd_topic,
            enable_viewer=True,
            reduction_ratio=0.2,
            safety_margin=0.02,
            **kwargs
    ):
        super(HPPManipulationInterface, self).__init__()

        loadServerPlugin("corbaserver", "manipulation-corba.so")
        Client().problem.resetProblem()

        self._rm = Model(robot_name, robot_pkg_name, robot_urdf_name, robot_srdf_name)
        self._om = Model(object_name, object_pkg_name, surface=object_surface, handle=object_handle)
        self._em = Model(env_name, env_pkg_name, surface=env_surface)
        self._gm = Gripper(self._rm, gripper_name, fingers, finger_joints, finger_joint_values)

        self.Modes = namedtuple("Modes", "idle approach grasp")
        self._current_mode = self.Modes.idle

        self._robot = self._create_robot()
        self._problem_solver = ProblemSolver(self._robot)

        self._problem_solver.loadPlugin('manipulation-spline-gradient-based.so')
        # To show available path optimizers:
        # ['EnforceTransitionSemantic', 'Graph-PartialShortcut', 'Graph-RandomShortcut', 'PartialShortcut',
        # 'RandomShortcut', 'SimpleShortcut', 'SimpleTimeParameterization', 'SplineGradientBased_bezier1',
        # 'SplineGradientBased_bezier3']
        # print(self._problem_solver.getAvailable('PathOptimizer'))
        self._problem_solver.addPathOptimizer('RandomShortcut')

        # To show available config validation methods
        # (['CollisionValidation', 'JointBoundValidation'])
        # print(self._problem_solver.getAvailable('ConfigValidation'))
        # self._problem_solver.clearConfigValidations()

        self._enable_viewer = enable_viewer
        self._viewer_factory = self._create_viewer_factory()  # need to create this no matter if enable_viewer

        self._robot.setJointBounds("{}/root_joint".format(self._rm.name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(self._om.name), object_bound)

        # An absolute value, if the threshold is surpassed, will raise the error `A configuration has no node`
        self._error_threshold = 3.e-3
        self._problem_solver.setErrorThreshold(self._error_threshold)
        self._problem_solver.setMaxIterProjection(80)

        # Use this one and/or the next to limit solving time. MaxIterPathPlanning is a fairly large value if not set.
        # self._problem_solver.setMaxIterPathPlanning(40)
        self._problem_solver.setTimeOutPathPlanning(20)

        # ['PathOptimizer', 'PathProjector', 'PathPlanner', 'ConfigurationShooter', 'PathValidation',
        #  'ConfigValidation', 'SteeringMethod', 'Distance', 'NumericalConstraint', 'CenterOfMass', 'Problem',
        #  'Parameter', 'DefaultParameter', 'Gripper', 'Handle', 'RobotContact', 'EnvContact', 'ConstraintGraph']
        # rospy.loginfo('Using path planner {}, available planners are: {}'.format(
        #     self._problem_solver.getSelected('PathPlanner'), self._problem_solver.getAvailable('PathPlanner')))

        # self._problem_solver.createTransformationConstraint("placement", '', "{}/root_joint".format(self._om.name),
        #                                                     [0, 0, 1, 0, 0, 0, 1],
        #                                                     [True, True, True, True, True, True])

        self._joint_names = []
        self._q_current = self._robot.getCurrentConfig()

        self._lock_hand = self._create_locks()

        self._graph_id = 0
        self._safety_margin = safety_margin
        self._constraint_graph = self._create_constraint_graph()

        self._joint_cmd_publisher = rospy.Publisher(joint_cmd_topic, JointState, queue_size=1)
        self._base_cmd_publisher = rospy.Publisher(base_cmd_topic, Twist, queue_size=1)

        self._time_step = 0.002
        self._reduction_ratio = reduction_ratio

        self._object_transform = transform.rotation_matrix(-np.pi / 2, (0, 1, 0))

        self._base_p_tol = 0.01
        self._base_o_tol = 0.02
        self._object_p_tol = 0.01
        self._object_o_tol = 0.02

    def _create_robot(self, root_joint_type='planar'):
        class CompositeRobot(Parent):
            urdfFilename = "package://{}/urdf/{}.urdf".format(self._rm.pkg_name, self._rm.urdf_name)
            srdfFilename = "package://{}/srdf/{}.srdf".format(self._rm.pkg_name, self._rm.srdf_name)
            rootJointType = root_joint_type

            # \param compositeName name of the composite robot that will be built later,
            # \param robotName name of the first robot that is loaded now,
            # \param load whether to actually load urdf files. Set to false if you only
            #        want to initialize a corba client to an already initialized problem.
            # \param rootJointType type of root joint among ("freeflyer", "planar", "anchor"),
            def __init__(self, composite_name, robot_name, load=True,
                         root_joint="planar", **kwargs):
                Parent.__init__(self, composite_name, robot_name, root_joint, load, **kwargs)

        robot = CompositeRobot('{}-{}'.format(self._rm.name, self._om.name), self._rm.name)
        return robot

    def _create_viewer_factory(self):
        object_to_grasp = Object(self._om.name, self._om.pkg_name)
        environment = Environment(self._em.name, self._em.pkg_name)
        viewer_factory = ViewerFactory(self._problem_solver)
        viewer_factory.loadObjectModel(object_to_grasp, self._om.name)
        viewer_factory.loadEnvironmentModel(environment, self._em.name)
        return viewer_factory

    def _create_locks(self):
        lock_hand = self._gm.fingers
        for i in range(self._gm.dof):
            self._problem_solver.createLockedJoint(
                self._gm.fingers[i], '{}/{}'.format(self._rm.name, self._gm.joints[i]), [self._gm.values[i], ]
            )
        return lock_hand

    def _create_constraint_graph(self, initialize=True):
        """When new graph is created, we increase the count of graph_id to prevent naming ambiguous.
        Creating multiple graphs only consume a little memory and beneficial for increase planning success rate.

        Args:
            initialize: bool If true, the created graph will be initialized.

        Returns:
            Created constraint graph.
        """
        self._graph_id += 1
        graph_name = 'graph_{}'.format(self._graph_id)
        graph = ConstraintGraph(self._robot, graph_name)
        rospy.logdebug('Created new constraint graph: {}'.format(graph_name))
        factory = ConstraintGraphFactory(graph)
        factory.setGrippers(["{}/{}".format(self._rm.name, self._gm.name), ])
        factory.environmentContacts(["{}/{}".format(self._em.name, self._em.surface), ])
        factory.setObjects([self._om.name, ],
                           [["{}/{}".format(self._om.name, self._om.handle), ], ],
                           [["{}/{}".format(self._om.name, self._om.surface), ], ])
        factory.setRules([Rule([".*"], [".*"], True), ])
        # factory.setPreplacementDistance('{}'.format(self._om.name), 0.1)
        factory.generate()
        graph.addConstraints(graph=True, constraints=Constraints(numConstraints=self._lock_hand))
        # graph.display(open=False)
        security_margins = SecurityMargins(self._problem_solver, factory, [self._rm.name, self._om.name, 'universe'])
        security_margins.setSecurityMarginBetween(self._rm.name, self._om.name, self._safety_margin)
        security_margins.apply()
        if initialize:
            graph.initialize()
        return graph

    def _update_constraint_graph(self):
        """Update the constraints according to working modes. In the grasp mode,
        we need to add extra constraints to the edges such that the base do not move during grasping.

        Returns:
            None
        """
        if self._current_mode == self.Modes.grasp:
            self._constraint_graph = self._create_constraint_graph(initialize=False)
            self._problem_solver.createLockedJoint("fix_base", "{}/root_joint".format(self._rm.name), [0, 0, 1, 0])
            for e in self._constraint_graph.edges.keys():
                self._constraint_graph.addConstraints(edge=e, constraints=Constraints(numConstraints=["fix_base"]))
            self._constraint_graph.initialize()
        else:
            self._constraint_graph = self._create_constraint_graph()

    def update_current_config(self, msg):
        if isinstance(msg, JointState):
            self._set_robot_joint_state_config(self._q_current, msg, add_name=True)
            rospy.loginfo_once('First joint state received')
        elif isinstance(msg, Odometry):
            self._set_planar_robot_base_config(self._q_current, msg)
            rospy.loginfo_once('First odom received')
        elif isinstance(msg, Pose):
            self._set_object_config(self._q_current, msg)
            rospy.loginfo_once('First object pose received')
        else:
            rospy.logerr("Msg is not of type JointState/Odometry/Pose: {}".format(type(msg)))

    def get_current_base_global_pose(self):
        return self._get_planar_robot_base_pose(self._q_current)

    def _implement_plan(self, q_goal):
        """Given the goal configuration and dynamically retrieved initial configuration, this function plans a
        valid path and implements the plan.

        Args:
            q_goal: list[double] Complete configuration of the goal state.

        Returns:
            bool True if succeeded, false otherwise.
        """
        q_init = self._regulate_lock_hand_joints(self._q_current)
        # q_init = self._project_config_if_necessary(self._q_current)
        self._problem_solver.addGoalConfig(q_goal)
        self._problem_solver.setInitialConfig(q_init)

        rospy.logdebug("Init configuration:\n{}".format(["{0:0.8f}".format(i) for i in q_init]))
        rospy.logdebug("Goal configuration:\n{}".format(["{0:0.8f}".format(i) for i in q_goal]))

        self._update_constraint_graph()
        try:
            time_spent = self._problem_solver.solve()
            rospy.loginfo('Plan solved in {}h-{}m-{}s-{}ms'.format(*time_spent))
        except BaseException as e:
            rospy.logwarn('Failed to solve due to {}'.format(e))
            self._problem_solver.resetGoalConfigs()
            return False

        if self._enable_viewer:
            viewer = self._viewer_factory.createViewer()
            viewer(q_init)
            path_player = PathPlayer(viewer)
            path_player(self._last_path_id)

        # Get waypoints information
        grasp_stamps = []
        release_stamps = []
        if self._current_mode == self.Modes.grasp:
            # waypoints is a list of configurations, each configuration corresponds to a node (or state)
            # in the graph, time stamp to reach that state is recorded in times.
            waypoints, times = self._problem_solver.getWaypoints(self._last_path_id)
            temp = [[wp, t] for wp, t in zip(waypoints, times)]
            for i in range(len(temp) - 1):
                if self._constraint_graph.getNode(temp[i][0]) == 'free' and \
                        'grasp' in self._constraint_graph.getNode(temp[i + 1][0]):
                    grasp_stamp = temp[i + 1][1]
                    grasp_stamps.append(grasp_stamp)
                    rospy.loginfo('Will grasp at {:.4f} s'.format(grasp_stamp))
                if self._constraint_graph.getNode(temp[i + 1][0]) == 'free' and \
                        'grasp' in self._constraint_graph.getNode(temp[i][0]):
                    release_stamps.append(temp[i + 1][1])
                    rospy.loginfo('Will release at {:.4f} s'.format(temp[i + 1][1]))

        path_length = self._problem_solver.pathLength(self._last_path_id)
        msgs = []
        close_gripper = False
        for t in np.arange(0, path_length, self._time_step):
            j_q = self._problem_solver.configAtParam(self._last_path_id, t)
            j_dq = self._problem_solver.derivativeAtParam(self._last_path_id, 1, t)
            for g_s in grasp_stamps:
                if t <= g_s < t + self._time_step:
                    close_gripper = True
                    break
            for r_s in release_stamps:
                if t <= r_s < t + self._time_step:
                    close_gripper = False
                    break
            msgs.append(self._derive_command_msgs(j_q, j_dq, close_gripper))
        j_q = self._problem_solver.configAtParam(self._last_path_id, path_length)
        j_dq = self._problem_solver.derivativeAtParam(self._last_path_id, 1, path_length)
        joint_cmd, base_cmd = self._derive_command_msgs(j_q, j_dq)

        global_start = time.time()
        r = rospy.Rate(1. / self._time_step * self._reduction_ratio)
        for j_cmd, base_cmd in msgs:
            self._publish_planning_results(j_cmd, base_cmd)
            r.sleep()

        self._publish_planning_results(joint_cmd, base_cmd)
        rospy.Rate(1. / (path_length % self._time_step) * self._reduction_ratio).sleep()
        rospy.loginfo('Path length: {:.4f}; actual elapsed time: {:.4f}'.format(
            path_length, (time.time() - global_start) * self._reduction_ratio))
        self._stop_base()
        self._problem_solver.resetGoalConfigs()
        return True

    def _project_config_if_necessary(self, config, node='free'):
        """Project the configuration to the given node of the constraint graph to get a valid configuration.

        Args:
            config: list[double] Complete configuration to project.
            node: str Name of the node.

        Returns:
            list[double] Projected configuration.
        """
        try:
            name = self._constraint_graph.getNode(config)
            assert name == node, print('{} is not {}'.format(name, node))
            return config
        except BaseException as e:
            rospy.logdebug(e)
            res, config_proj, _ = self._constraint_graph.applyNodeConstraints(node, config)
            self._log_config("Original configuration", config, show_all=True)
            self._log_config("Projected configuration", config_proj, show_all=True)
            if res:
                return config_proj
            else:
                raise ValueError('Failed to project config into {} node'.format(node))

    def _regulate_lock_hand_joints(self, config):
        """Manually set the locked hand joints the values defined with the constraint.

        Args:
            config: list[double] Complete configuration.

        Returns:
            list[double] Regulated configuration.
        """
        regulated_configs = config[::]
        for joint_name in self._joint_names:
            if joint_name in self._gm.joints:
                rank = self._robot.rankInConfiguration['{}/{}'.format(self._rm.name, joint_name)]
                idx = self._gm.joints.index(joint_name)
                regulated_configs[rank] = self._gm.values[idx]
        return regulated_configs

    def approach(self, base_goal_pose, joint_goal_state, base_pos_tol, base_ori_tol):
        """Make a plan for the robot to approach the given base goal pose and joint goal state.

        Args:
            base_goal_pose: Pose Goal pose of the base in the global frame.
            joint_goal_state: JointState Goal joint state of the robot joints.
            base_pos_tol: double Position tolerance of the base.
            base_ori_tol: double Orientation tolerance of the base.

        Returns:
            bool If success, return True, False otherwise.
        """
        q_goal = self._regulate_lock_hand_joints(self._q_current)
        # q_goal = self._project_config_if_necessary(self._q_current[::])
        goal_odom = Odometry()
        goal_odom.pose.pose = base_goal_pose
        self._set_planar_robot_base_config(q_goal, goal_odom)
        self._set_robot_joint_state_config(q_goal, joint_goal_state)
        self._base_p_tol = base_pos_tol if base_pos_tol > 0 else self._base_p_tol
        self._base_o_tol = base_ori_tol if base_ori_tol > 0 else self._base_o_tol
        self._current_mode = self.Modes.approach
        while not self._check_approaching_goal_reached(q_goal):
            if not self._implement_plan(q_goal):
                return False
        self._current_mode = self.Modes.idle
        return True

    def grasp(self, joint_goal_state, object_goal_pose, object_pos_tol, object_ori_tol):
        """Make a grasping plan.

        Args:
            joint_goal_state: JointState Goal joint state of the robot joints.
            object_goal_pose: Pose Goal pose of the object in the global frame.
            object_pos_tol: double Position tolerance of the object.
            object_ori_tol: double Orientation tolerance of the object.

        Returns:
            True if succeed, False otherwise.
        """
        q_goal = self._regulate_lock_hand_joints(self._q_current)
        # q_goal = self._project_config_if_necessary(self._q_current[::])
        # TODO Since we fix the base during grasping, the base goal is currently disabled.
        # It shall be activated if we have accurate locomotion.
        # odom = Odometry()
        # odom.pose.pose = base_goal_pose
        # self._set_robot_base_config(self._q_goal, odom)
        # self._base_p_tol = base_pos_tol if base_pos_tol > 0 else self._base_p_tol
        # self._base_o_tol = base_ori_tol if base_ori_tol > 0 else self._base_o_tol
        self._set_robot_joint_state_config(q_goal, joint_goal_state)
        self._set_object_config(q_goal, object_goal_pose)
        self._object_p_tol = object_pos_tol if object_pos_tol > 0 else self._object_p_tol
        self._object_o_tol = object_ori_tol if object_ori_tol > 0 else self._object_o_tol
        self._current_mode = self.Modes.grasp
        while not self._check_grasping_goal_reached(q_goal):
            # During grasping, the robot base could move due to external perturbations or the robot hitting an
            # obstacle, so we need to make sure the locomotion is within the tolerance, otherwise the initial and
            # goal configurations will never be connected together. If the base has moved, we use the new base pose
            # as the goal pose.
            if not self._check_approaching_goal_reached(q_goal, self._error_threshold):
                q_goal[:4] = self._q_current[:4]

            if not self._implement_plan(q_goal):
                return False
        self._current_mode = self.Modes.idle
        return True

    @property
    def _last_path_id(self):
        return self._problem_solver.numberPaths() - 1

    @staticmethod
    def _get_planar_robot_base_pose(config):
        pose = Pose()
        pose.position.x = config[0]
        pose.position.y = config[1]
        cos_yaw = config[2]
        sin_yaw = config[3]
        yaw = math.atan2(sin_yaw, cos_yaw)
        pose.orientation = common.to_ros_orientation(transform.euler_matrix(yaw, 0, 0, 'szyx'))
        return pose

    def _set_robot_joint_state_config(self, config, joint_state, add_name=False):
        """Update the configuration corresponding to the robot's actuated joints using the given joint state msg.
        The configuration will only be updated if the msg contains a name matching the predefined joint name.
        We allow undocumented names inside the msg, which will be ignored.

        Args:
            config:  list[double] Complete configuration to be updated in-place.
            joint_state: JointState Joint state msg of the robot.
            add_name: bool Whether adding names from the msg to class member _joint_names.
                      This allows only the measured joint states to set the names, but not allow the goal to set them.

        Returns:
            None
        """
        assert isinstance(joint_state, JointState)
        for i, joint_name in enumerate(joint_state.name):
            try:
                rank = self._robot.rankInConfiguration['{}/{}'.format(self._rm.name, joint_name)]
                config[rank] = joint_state.position[i]
                if add_name and joint_name not in self._joint_names:
                    self._joint_names.append(joint_name)
            except KeyError:
                continue

    @staticmethod
    def _set_planar_robot_base_config(config, base_odom):
        """Update the configuration corresponding to the robot base's pose using the given odom msg.

        Args:
            config: list[double] Complete configuration to be updated in-place.
            base_odom: Odometry Pose msg of the robot's planar base.

        Returns:
            None
        """
        assert isinstance(base_odom, Odometry)
        base_pose = base_odom.pose.pose
        config[0:2] = [base_pose.position.x, base_pose.position.y]
        yaw = transform.euler_from_matrix(common.to_orientation_matrix(base_pose.orientation), 'szyx')[0]
        config[2:4] = [math.cos(yaw), math.sin(yaw)]

    def _set_object_config(self, config, object_pose):
        """According to the convention of HPP manipulation, the object's local frame should have its
        x-axis be antipodal with the gripper's x-axis, which points from the gripper to the object.

        Args:
            config: list[double] The HPP configuration to update.
            object_pose: Pose Object pose derived from ROS msg.

        Returns:
            None
        """
        assert isinstance(object_pose, Pose)
        rank = self._robot.rankInConfiguration['{}/root_joint'.format(self._om.name)]
        object_pose.position.z += 0.002  # Avoid false positive collision due to measurement error
        # Rotate around the y-axis of the object frame to make it X-up.
        new_pose_matrix = np.dot(common.sd_pose(object_pose), self._object_transform)
        config[rank: rank + 7] = common.to_list(common.to_ros_pose(new_pose_matrix))

    def _publish_planning_results(self, joint_cmd, base_cmd):
        if self._current_mode == self.Modes.grasp:
            self._stop_base()
            self._joint_cmd_publisher.publish(joint_cmd)
        else:
            self._base_cmd_publisher.publish(base_cmd)
            self._joint_cmd_publisher.publish(joint_cmd)

    def _derive_command_msgs(self, j_q, j_dq, close_gripper=False):
        """Derive command msgs for robot joints and the base.

        Args:
            j_q: list[double] Joint position commands.
            j_dq: list[double] Joint velocity commands.
            close_gripper: bool If true, the gripper joints' commands will be set as 0.
                           Otherwise, the commands will follow the values in j_q.

        Returns:
            list[JointState, Twist] Commands for the joints and the base.
        """
        joint_cmd = JointState()
        for name in self._joint_names:
            joint_cmd.name.append(name)
            rank = self._robot.rankInConfiguration['{}/{}'.format(self._rm.name, name)]
            if name in self._gm.joints and close_gripper:
                # TODO the 0 value may fail in different grasping scenarios
                joint_cmd.position.append(0)
            else:
                joint_cmd.position.append(j_q[rank])
            rank = self._robot.rankInVelocity['{}/{}'.format(self._rm.name, name)]
            joint_cmd.velocity.append(j_dq[rank])
            joint_cmd.effort.append(0)  # TODO currently torque control is not supported

        base_cmd = Twist()
        rotation_2d = np.array([[j_q[2], -j_q[3]], [j_q[3], j_q[2]]])
        local_linear = np.dot(rotation_2d.T, np.array([j_dq[0], j_dq[1]]).T)
        base_cmd.linear.x = local_linear[0] * self._reduction_ratio
        base_cmd.linear.y = local_linear[1] * self._reduction_ratio
        base_cmd.angular.z = j_dq[2] * self._reduction_ratio
        return [joint_cmd, base_cmd]

    def _stop_base(self):
        base_cmd = Twist()
        self._base_cmd_publisher.publish(base_cmd)

    def _check_approaching_goal_reached(self, q_goal, base_tol=None):
        """Check if the base and the actuated joints have reached the goal.

        Args:
            q_goal list[double] Complete goal configuration to be reached.
            base_tol None/double Tolerance of the base error.

        Returns:
            True if both position and orientation errors are within tolerance.
        """
        base_curr_pos = self._q_current[:2]
        base_curr_ori = self._q_current[2:4]
        base_goal_pos = q_goal[:2]
        base_goal_ori = q_goal[2:4]
        joint_curr_cfg = self._get_joint_configs(self._q_current)
        joint_goal_cfg = self._get_joint_configs(q_goal)
        if isinstance(base_tol, float) and base_tol > 0:
            return common.all_close(base_goal_pos, base_curr_pos, base_tol) & \
                   common.all_close(base_goal_ori, base_curr_ori, base_tol) & \
                   common.all_close(joint_curr_cfg, joint_goal_cfg, 0.1)
        else:
            return common.all_close(base_goal_pos, base_curr_pos, self._base_p_tol) & \
                   common.all_close(base_goal_ori, base_curr_ori, self._base_o_tol) & \
                   common.all_close(joint_curr_cfg, joint_goal_cfg, 0.1)

    def _check_grasping_goal_reached(self, q_goal):
        """Check if the object being manipulated has reached the goal pose.

        Returns:
            True if both position and orientation errors are within tolerance.
        """
        assert self._current_mode == self.Modes.grasp
        rank = self._robot.rankInConfiguration['{}/root_joint'.format(self._om.name)]
        obj_curr_pos = self._q_current[rank: rank + 3]
        obj_curr_ori = self._q_current[rank + 3: rank + 7]
        obj_goal_pos = q_goal[rank: rank + 3]
        obj_goal_ori = q_goal[rank + 3: rank + 7]
        return common.all_close(obj_goal_pos, obj_curr_pos, self._object_p_tol) & \
               common.all_close(obj_goal_ori, obj_curr_ori, self._object_o_tol)

    def _get_joint_configs(self, all_configs):
        joint_configs = []
        for joint_name in self._joint_names:
            try:
                rank = self._robot.rankInConfiguration['{}/{}'.format(self._rm.name, joint_name)]
                joint_configs.append(all_configs[rank])
            except KeyError:
                continue
        return joint_configs

    def _log_config(self, title, config, show_all=False):
        rospy.loginfo("{}\n".format(title))
        if show_all:
            rospy.loginfo("{}".format(["{:0.19f}".format(i) for i in config]))
            return

        joint_states = self._get_joint_configs(config)
        max_key_len = len(max(self._joint_names, key=len))
        for name, state in zip(self._joint_names, joint_states):
            name_padded = '{message: <{width}}'.format(message=name, width=max_key_len)
            rospy.loginfo("{}: {:0.19f}".format(name_padded, state))
