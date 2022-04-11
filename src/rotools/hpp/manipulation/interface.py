from __future__ import print_function

import math
import numpy as np

import rospy

import hpp_idl
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
            **kwargs
    ):
        super(HPPManipulationInterface, self).__init__()

        loadServerPlugin("corbaserver", "manipulation-corba.so")
        Client().problem.resetProblem()

        self._rm = Model(robot_name, robot_pkg_name, robot_urdf_name, robot_srdf_name)
        self._om = Model(object_name, object_pkg_name, surface=object_surface, handle=object_handle)
        self._em = Model(env_name, env_pkg_name, surface=env_surface)
        self._gm = Gripper(self._rm, gripper_name, fingers, finger_joints, finger_joint_values)

        self._robot = self._create_robot()
        self._problem_solver = ProblemSolver(self._robot)

        self._enable_viewer = enable_viewer
        self._viewer_factory = self._create_viewer_factory()  # need to create this no matter if enable_viewer

        self._robot.setJointBounds("{}/root_joint".format(self._rm.name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(self._om.name), object_bound)

        # robot.client.basic.problem.resetRoadmap ()
        self._problem_solver.setErrorThreshold(1e-2)
        self._problem_solver.setMaxIterProjection(40)

        # self._problem_solver.createTransformationConstraint("placement", '', "cube_30/root_joint",
        #                                                     [0, 0, 1, 0, 0, 0, 1],
        #                                                     [True, True, True, True, True, True])

        self._joint_names = []
        self._q_current = self._robot.getCurrentConfig()
        self._q_goal = self._q_current[::]

        self._lock_hand = self._create_lock_hand()

        self._constrain_graph = self._create_constrain_graph()

        self._joint_cmd_publisher = rospy.Publisher(joint_cmd_topic, JointState, queue_size=1)
        self._base_cmd_publisher = rospy.Publisher(base_cmd_topic, Twist, queue_size=1)

        self._time_step = 0.01
        self._reduction_ratio = 0.5

        self._object_transform = transform.rotation_matrix(-np.pi / 2, (0, 1, 0))

    def _create_robot(self, root_joint_type='planar'):
        class CompositeRobot(Parent):
            urdfFilename = "package://{}/urdf/{}.urdf".format(self._rm.pkg_name, self._rm.urdf_name)
            srdfFilename = "package://{}/srdf/{}.srdf".format(self._rm.pkg_name, self._rm.srdf_name)
            rootJointType = root_joint_type

            # \param compositeName name of the composite robot that will be built later,
            # \param robotName name of the first robot that is loaded now,
            # \param load whether to actually load urdf files. Set to false if you only
            #        want to initialize a corba client to an already initialized
            #        problem.
            # \param rootJointType type of root joint among ("freeflyer", "planar",
            #        "anchor"),
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

    def _create_lock_hand(self):
        lock_hand = self._gm.fingers
        for i in range(self._gm.dof):
            self._problem_solver.createLockedJoint(
                self._gm.fingers[i], '{}/{}'.format(self._rm.name, self._gm.joints[i]), [self._gm.values[i], ]
            )
        return lock_hand

    def _create_constrain_graph(self):
        constrain_graph = ConstraintGraph(self._robot, 'graph')
        factory = ConstraintGraphFactory(constrain_graph)
        factory.setGrippers(["{}/{}".format(self._rm.name, self._gm.name), ])
        factory.environmentContacts(["{}/{}".format(self._em.name, self._em.surface), ])
        factory.setObjects([self._om.name, ],
                           [["{}/{}".format(self._om.name, self._om.handle), ], ],
                           [["{}/{}".format(self._om.name, self._om.surface), ], ])
        factory.setRules([Rule([".*"], [".*"], True), ])
        factory.generate()
        constrain_graph.addConstraints(graph=True, constraints=Constraints(numConstraints=self._lock_hand))
        constrain_graph.initialize()
        return constrain_graph

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

    def _make_plan(self, base_pos_tol, base_ori_tol, object_pos_tol=None, object_ori_tol=None):
        # res, q_goal_proj, err = self._constrain_graph.applyNodeConstraints("free", self._q_goal)
        # self._problem_solver.addGoalConfig(q_goal_proj)
        self._problem_solver.addGoalConfig(self._q_goal)

        while not self._check_goal_reached(base_pos_tol, base_ori_tol, object_pos_tol, object_ori_tol):
            # res, q_init_proj, err = self._constrain_graph.applyNodeConstraints("free", self._q_current)
            # self._problem_solver.setInitialConfig(q_init_proj)
            self._problem_solver.setInitialConfig(self._q_current)

            # rospy.loginfo("Goal configuration:\n{}".format(["{0:0.2f}".format(i) for i in q_goal_proj]))
            # rospy.loginfo("Current configuration:\n{}".format(["{0:0.2f}".format(i) for i in q_init_proj]))
            rospy.logdebug("Goal configuration:\n{}".format(["{0:0.2f}".format(i) for i in self._q_goal]))
            rospy.logdebug("Current configuration:\n{}".format(["{0:0.2f}".format(i) for i in self._q_current]))

            try:
                time_spent = self._problem_solver.solve()
                rospy.loginfo('Plan solved in {}h-{}m-{}s-{}ms'.format(*time_spent))
            except BaseException as e:
                rospy.logwarn('Failed to solve due to {}'.format(e))
                self._problem_solver.resetGoalConfigs()
                return False

            self._problem_solver.optimizePath(self._last_path_id)

            if self._enable_viewer:
                viewer = self._viewer_factory.createViewer()
                # viewer(q_init_proj)
                viewer(self._q_current)
                path_player = PathPlayer(viewer)
                path_player(self._last_path_id)

            path_length = self._problem_solver.pathLength(self._last_path_id)
            r = rospy.Rate(1. / self._time_step * self._reduction_ratio)
            for t in np.arange(0, path_length, self._time_step):
                j_q = self._problem_solver.configAtParam(self._last_path_id, t)
                j_dq = self._problem_solver.derivativeAtParam(self._last_path_id, 1, t)
                self._publish_planning_results(j_q, j_dq, base_pos_tol, base_ori_tol)
                r.sleep()

            r_final = rospy.Rate(1. / (path_length % self._time_step) * self._reduction_ratio)
            j_q = self._problem_solver.configAtParam(self._last_path_id, path_length)
            j_dq = self._problem_solver.derivativeAtParam(self._last_path_id, 1, path_length)
            self._publish_planning_results(j_q, j_dq, base_pos_tol, base_ori_tol)
            r_final.sleep()
            self._stop_base()

        self._problem_solver.resetGoalConfigs()
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
        return self._make_plan(base_pos_tol, base_ori_tol)

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
        odom = Odometry()
        odom.pose.pose = base_goal_pose
        self._set_robot_base_config(self._q_goal, odom)
        self._set_robot_joint_state_config(self._q_goal, joint_goal_state)
        self._set_object_config(self._q_goal, object_goal_pose)
        return self._make_plan(base_pos_tol, base_ori_tol, object_pos_tol, object_ori_tol)

    def _make_grasping_plan(self, pos_tol, ori_tol):
        res, q_init_proj, err = self._constrain_graph.applyNodeConstraints("free", self._q_current)
        self._problem_solver.setInitialConfig(q_init_proj)

        rospy.loginfo("Current location:\n{}".format(q_init_proj))

        self._problem_solver.setTargetState(self._constrain_graph.nodes[
                                                "{}/{} grasps {}/{}".format(self._rm.name, self._gm.name,
                                                                            self._om.name, self._om.handle)])
        time_spent = self._problem_solver.solve()
        rospy.loginfo('Grasping plan solved in {}h-{}m-{}s-{}ms'.format(*time_spent))

        self._problem_solver.optimizePath(self._last_path_id)

        if self._enable_viewer:
            viewer = self._viewer_factory.createViewer()
            viewer(q_init_proj)
            path_player = PathPlayer(viewer)
            path_player(self._last_path_id)

        path_length = self._problem_solver.pathLength(self._last_path_id)
        r = rospy.Rate(1. / self._time_step * self._reduction_ratio)
        for t in np.arange(0, path_length, self._time_step):
            j_q = self._problem_solver.configAtParam(self._last_path_id, t)
            j_dq = self._problem_solver.derivativeAtParam(self._last_path_id, 1, t)
            self._publish_planning_results(j_q, j_dq, pos_tol, ori_tol, check=False)
            r.sleep()

        j_q = self._problem_solver.configAtParam(self._last_path_id, path_length)
        j_dq = self._problem_solver.derivativeAtParam(self._last_path_id, 1, path_length)
        self._publish_planning_results(j_q, j_dq, pos_tol, ori_tol, check=False)
        self._stop_base()

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

    def _publish_planning_results(self, j_q, j_dq, base_pos_tol, base_ori_tol, check=True):
        joint_cmd = JointState()
        for name in self._joint_names:
            joint_cmd.name.append(name)
            rank = self._robot.rankInConfiguration['{}/{}'.format(self._rm.name, name)]
            joint_cmd.position.append(j_q[rank])
            rank = self._robot.rankInVelocity['{}/{}'.format(self._rm.name, name)]
            joint_cmd.velocity.append(j_dq[rank])
            joint_cmd.effort.append(0)  # TODO currently torque control is not supported
        self._joint_cmd_publisher.publish(joint_cmd)

        base_cmd = Twist()
        rotation_2d = np.array([[j_q[2], -j_q[3]], [j_q[3], j_q[2]]])
        local_linear = np.dot(rotation_2d.T, np.array([j_dq[0], j_dq[1]]).T)
        base_cmd.linear.x = local_linear[0] * self._reduction_ratio
        base_cmd.linear.y = local_linear[1] * self._reduction_ratio
        base_cmd.angular.z = j_dq[2] * self._reduction_ratio * 1.2  # 1.2 is an empirical value
        if check:
            if not self._check_goal_reached(base_pos_tol, base_ori_tol):
                self._base_cmd_publisher.publish(base_cmd)
            else:
                self._stop_base()
        else:
            self._base_cmd_publisher.publish(base_cmd)

    def _stop_base(self):
        base_cmd = Twist()
        self._base_cmd_publisher.publish(base_cmd)

    def _check_goal_reached(self, base_pos_tol, base_ori_tol, object_pos_tol=None, object_ori_tol=None):
        """Check if the object and optionally the base has reached the goal pose.

        Args:
            base_pos_tol: double Base position tolerance.
            base_ori_tol: double Base orientation tolerance.
            object_pos_tol: double Base position tolerance.
            object_ori_tol: double Base orientation tolerance.

        Returns:
            True if both position and orientation errors are within tolerance.
        """
        base_goal_pos = self._q_goal[:2]
        base_curr_pos = self._q_current[:2]
        base_goal_ori = self._q_goal[2:4]
        base_curr_ori = self._q_current[2:4]
        if object_pos_tol is None or object_ori_tol is None:
            return common.all_close(base_goal_pos, base_curr_pos, base_pos_tol) & \
                   common.all_close(base_goal_ori, base_curr_ori, base_ori_tol)

        rank = self._robot.rankInConfiguration['{}/root_joint'.format(self._om.name)]
        obj_curr_pos = self._q_current[rank: rank + 3]
        obj_goal_pos = self._q_goal[rank: rank + 3]
        obj_curr_ori = self._q_current[rank + 3: rank + 7]
        obj_goal_ori = self._q_goal[rank + 3: rank + 7]
        return common.all_close(obj_goal_pos, obj_curr_pos, object_pos_tol) & \
               common.all_close(obj_goal_ori, obj_curr_ori, object_ori_tol) & \
               common.all_close(base_goal_pos, base_curr_pos, base_pos_tol) & \
               common.all_close(base_goal_ori, base_curr_ori, base_ori_tol)
