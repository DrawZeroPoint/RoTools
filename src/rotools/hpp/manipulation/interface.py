from __future__ import print_function

import math
import time

import numpy as np
from collections import namedtuple

import rospy

from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule, \
    Constraints, ConstraintGraphFactory, Client
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
            **kwargs
    ):
        super(HPPManipulationInterface, self).__init__()

        loadServerPlugin("corbaserver", "manipulation-corba.so")
        Client().problem.resetProblem()

        self._rm = Model(robot_name, robot_pkg_name, robot_urdf_name, robot_srdf_name)
        self._om = Model(object_name, object_pkg_name, surface=object_surface, handle=object_handle)
        self._em = Model(env_name, env_pkg_name, surface=env_surface)
        self._gm = Gripper(self._rm, gripper_name, fingers, finger_joints, finger_joint_values)

        self._work_modes = namedtuple("work_modes", "idle approach grasp")
        self._previous_mode = self._work_modes.idle
        self._mode = self._work_modes.idle

        self._robot = self._create_robot()
        self._problem_solver = ProblemSolver(self._robot)

        self._problem_solver.loadPlugin('manipulation-spline-gradient-based.so')
        # To show available path optimizers:
        # print(self._problem_solver.getAvailable('PathOptimizer'))
        self._problem_solver.addPathOptimizer('RandomShortcut')

        self._enable_viewer = enable_viewer
        self._viewer_factory = self._create_viewer_factory()  # need to create this no matter if enable_viewer

        self._robot.setJointBounds("{}/root_joint".format(self._rm.name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(self._om.name), object_bound)

        # An absolute value, if the threshold is surpassed, will raise the error `A configuration has no node`
        self._problem_solver.setErrorThreshold(3e-3)
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
        self._q_goal = self._q_current[::]

        self._lock_hand = self._create_locks()

        self._graph_id = 0
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

    def _create_constraint_graph(self):
        self._graph_id += 1
        graph_name = 'graph_{}'.format(self._graph_id)
        constraint_graph = ConstraintGraph(self._robot, graph_name)
        rospy.loginfo('Created new constraint graph: {}'.format(graph_name))
        factory = ConstraintGraphFactory(constraint_graph)
        factory.setGrippers(["{}/{}".format(self._rm.name, self._gm.name), ])
        factory.environmentContacts(["{}/{}".format(self._em.name, self._em.surface), ])
        factory.setObjects([self._om.name, ],
                           [["{}/{}".format(self._om.name, self._om.handle), ], ],
                           [["{}/{}".format(self._om.name, self._om.surface), ], ])
        factory.setRules([Rule([".*"], [".*"], True), ])
        # factory.setPreplacementDistance('{}'.format(self._om.name), 0.1)
        factory.generate()
        constraint_graph.addConstraints(graph=True, constraints=Constraints(numConstraints=self._lock_hand))
        constraint_graph.initialize()
        return constraint_graph

    def _update_constraints(self):
        """Update the constraints if 1) Working mode turned from other modes to the grasp mode. In this mode,
        we need to add extra constraints to the edges such that the base do not move during grasping. 2) Working
        mode turned from grasping to other modes, where we need to create a new graph since the original constrained
        graph cannot be used. When new graph is created, we increase the count of graph_id to prevent naming ambiguous.

        Returns:
            None
        """
        if self._previous_mode == self._work_modes.grasp and self._mode != self._work_modes.grasp:
            self._constraint_graph = self._create_constraint_graph()
        if self._previous_mode != self._work_modes.grasp and self._mode == self._work_modes.grasp:
            self._problem_solver.createLockedJoint("fix_base", "{}/root_joint".format(self._rm.name), [0, 0, 1, 0])
            for e in self._constraint_graph.edges.keys():
                self._constraint_graph.addConstraints(edge=e, constraints=Constraints(numConstraints=["fix_base"]))
            self._constraint_graph.initialize()
        self._previous_mode = self._mode

    def update_current_config(self, msg):
        if isinstance(msg, JointState):
            self._set_robot_joint_state_config(self._q_current, msg, add_name=True)
            rospy.loginfo_once('First joint state received')
        elif isinstance(msg, Odometry):
            self._set_robot_base_config(self._q_current, msg)
            rospy.loginfo_once('First odom received')
        elif isinstance(msg, Pose):
            self._set_object_config(self._q_current, msg)
            rospy.loginfo_once('First object pose received')
        else:
            rospy.logerr("Msg is not of type JointState/Odometry/Pose: {}".format(type(msg)))

    def get_current_base_global_pose(self):
        return self._get_robot_base_pose(self._q_current)

    def _make_plan(self):
        while not self._check_goal_reached():
            q_goal = self._q_goal[::]
            q_current = self._q_current[::]
            if self._mode == self._work_modes.grasp:
                q_goal[:4] = q_current[:4]

            self._problem_solver.addGoalConfig(q_goal)
            self._problem_solver.setInitialConfig(q_current)

            rospy.loginfo("Current configuration:\n{}".format(["{0:0.8f}".format(i) for i in q_current]))
            rospy.loginfo("Goal configuration:\n{}".format(["{0:0.8f}".format(i) for i in q_goal]))

            self._update_constraints()
            try:
                time_spent = self._problem_solver.solve()
                rospy.loginfo('Plan solved in {}h-{}m-{}s-{}ms'.format(*time_spent))
            except BaseException as e:
                rospy.logwarn('Failed to solve due to {}'.format(e))
                self._problem_solver.resetGoalConfigs()
                self._mode = self._work_modes.idle
                return False

            if self._enable_viewer:
                viewer = self._viewer_factory.createViewer()
                viewer(q_current)
                path_player = PathPlayer(viewer)
                path_player(self._last_path_id)

            # Get waypoints information
            grasp_stamps = []
            release_stamps = []
            if self._mode == self._work_modes.grasp:
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

        self._mode = self._work_modes.idle
        return True

    def make_approaching_plan(self, base_goal_pose, joint_goal_state, base_pos_tol, base_ori_tol):
        """Make a plan for the robot to approach the given base goal pose and joint goal state.

        Args:
            base_goal_pose: Pose Goal pose of the base in the global frame.
            joint_goal_state: JointState Goal joint state of the robot joints.
            base_pos_tol: double Position tolerance of the base.
            base_ori_tol: double Orientation tolerance of the base.

        Returns:
            bool If success, return True, False otherwise.
        """
        self._q_goal = self._q_current[::]
        odom = Odometry()
        odom.pose.pose = base_goal_pose
        self._set_robot_base_config(self._q_goal, odom)
        self._set_robot_joint_state_config(self._q_goal, joint_goal_state)
        self._base_p_tol = base_pos_tol if base_pos_tol > 0 else self._base_p_tol
        self._base_o_tol = base_ori_tol if base_ori_tol > 0 else self._base_o_tol
        self._mode = self._work_modes.approach
        return self._make_plan()

    def make_grasping_plan(self, base_goal_pose, joint_goal_state, object_goal_pose, base_pos_tol, base_ori_tol,
                           object_pos_tol, object_ori_tol):
        """Make a grasping plan.

        Args:
            base_goal_pose: Pose Goal pose of the base in the global frame.
            joint_goal_state: JointState Goal joint state of the robot joints.
            object_goal_pose: Pose Goal pose of the object in the global frame.
            base_pos_tol: double Position tolerance of the base.
            base_ori_tol: double Orientation tolerance of the base.
            object_pos_tol: double Position tolerance of the object.
            object_ori_tol: double Orientation tolerance of the object.

        Returns:
            True if success, False otherwise.
        """
        self._q_goal = self._q_current[::]
        # TODO Since we fix the base during grasping, the base goal is currently disabled.
        # It shall be activated if we have accurate locomotion.
        # odom = Odometry()
        # odom.pose.pose = base_goal_pose
        # self._set_robot_base_config(self._q_goal, odom)
        # self._base_p_tol = base_pos_tol if base_pos_tol > 0 else self._base_p_tol
        # self._base_o_tol = base_ori_tol if base_ori_tol > 0 else self._base_o_tol
        self._set_robot_joint_state_config(self._q_goal, joint_goal_state)
        self._set_object_config(self._q_goal, object_goal_pose)
        self._object_p_tol = object_pos_tol if object_pos_tol > 0 else self._object_p_tol
        self._object_o_tol = object_ori_tol if object_ori_tol > 0 else self._object_o_tol
        self._mode = self._work_modes.grasp
        return self._make_plan()

    @property
    def _last_path_id(self):
        return self._problem_solver.numberPaths() - 1

    @staticmethod
    def _get_robot_base_pose(config):
        pose = Pose()
        pose.position.x = config[0]
        pose.position.y = config[1]
        cos_yaw = config[2]
        sin_yaw = config[3]
        yaw = math.atan2(sin_yaw, cos_yaw)
        pose.orientation = common.to_ros_orientation(transform.euler_matrix(yaw, 0, 0, 'szyx'))
        return pose

    def _set_robot_joint_state_config(self, config, joint_state, add_name=False):
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
    def _set_robot_base_config(config, base_odom):
        """Update the configurations of the robot base's pose.

        Args:
            config: list[double] HPP configurations.
            base_odom: Odometry msg of the robot's planar base.

        Returns:
            Updated configurations.
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
        if self._mode == self._work_modes.grasp:
            if not self._check_goal_reached():
                self._joint_cmd_publisher.publish(joint_cmd)
                self._base_cmd_publisher.publish(base_cmd)
            else:
                self._joint_cmd_publisher.publish(joint_cmd)
                self._stop_base()
        else:
            self._joint_cmd_publisher.publish(joint_cmd)
            self._base_cmd_publisher.publish(base_cmd)

    def _derive_command_msgs(self, j_q, j_dq, close_gripper=False):
        joint_cmd = JointState()
        for name in self._joint_names:
            joint_cmd.name.append(name)
            rank = self._robot.rankInConfiguration['{}/{}'.format(self._rm.name, name)]
            if name in self._gm.joints and close_gripper:
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

    def _check_goal_reached(self):
        """Check if the object and optionally the base has reached the goal pose.

        Returns:
            True if both position and orientation errors are within tolerance.
        """
        if self._mode == self._work_modes.approach:
            base_goal_pos = self._q_goal[:2]
            base_curr_pos = self._q_current[:2]
            base_goal_ori = self._q_goal[2:4]
            base_curr_ori = self._q_current[2:4]
            return common.all_close(base_goal_pos, base_curr_pos, self._base_p_tol) & \
                   common.all_close(base_goal_ori, base_curr_ori, self._base_o_tol)
        elif self._mode == self._work_modes.grasp:
            rank = self._robot.rankInConfiguration['{}/root_joint'.format(self._om.name)]
            obj_curr_pos = self._q_current[rank: rank + 3]
            obj_goal_pos = self._q_goal[rank: rank + 3]
            obj_curr_ori = self._q_current[rank + 3: rank + 7]
            obj_goal_ori = self._q_goal[rank + 3: rank + 7]
            return common.all_close(obj_goal_pos, obj_curr_pos, self._object_p_tol) & \
                   common.all_close(obj_goal_ori, obj_curr_ori, self._object_o_tol)
        else:
            raise NotImplementedError
