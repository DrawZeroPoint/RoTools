from __future__ import print_function

import math
import numpy as np

import rospy

from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import (
    ProblemSolver,
    ConstraintGraph,
    Rule,
    Constraints,
    ConstraintGraphFactory,
    Client,
)
from hpp.corbaserver import loadServerPlugin

from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory

from geometry_msgs.msg import Pose
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
        self._om = Model(
            object_name, object_pkg_name, surface=object_surface, handle=object_handle
        )
        self._em = Model(env_name, env_pkg_name, surface=env_surface)
        self._gm = Gripper(
            self._rm, gripper_name, fingers, finger_joints, finger_joint_values
        )

        self._robot = self._create_robot()
        self._problem_solver = ProblemSolver(self._robot)

        if enable_viewer:
            self._viewer_factory = self._create_viewer_factory()
        else:
            self._viewer_factory = None

        self._robot.setJointBounds("{}/root_joint".format(robot_name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(object_name), object_bound)

        # robot.client.basic.problem.resetRoadmap ()
        self._problem_solver.setErrorThreshold(1e-3)
        self._problem_solver.setMaxIterProjection(40)

        self._joint_names = []
        # Generate initial and goal configuration.
        self._q_current = self._robot.getCurrentConfig()

        self._q_current[0:2] = [-3.2, -4]
        rank = self._robot.rankInConfiguration[
            "{}/{}".format(self._rm.name, self._gm.joints[0])
        ]
        self._q_current[rank] = 0.5
        rank = self._robot.rankInConfiguration[
            "{}/{}".format(self._rm.name, self._gm.joints[1])
        ]
        self._q_current[rank] = 0.5
        rank = self._robot.rankInConfiguration[
            "{}/torso_lift_joint".format(self._rm.name)
        ]
        self._q_current[rank] = 0.2

        rank = self._robot.rankInConfiguration["{}/root_joint".format(self._om.name)]
        self._q_current[rank : rank + 3] = [-2.5, -4, 0.8]

        self._q_goal = self._q_current[::]
        self._q_goal[0:2] = [-3.2, -4]
        rank = self._robot.rankInConfiguration["{}/root_joint".format(self._om.name)]
        self._q_goal[rank : rank + 3] = [-3.5, -4, 0.8]

        self._lock_hand = self._create_lock_hand()

        self._constrain_graph = self._create_constrain_graph()

    def _create_robot(self, root_joint_type="planar"):
        class CompositeRobot(Parent):
            urdfFilename = "package://{}/urdf/{}.urdf".format(
                self._rm.pkg_name, self._rm.urdf_name
            )
            srdfFilename = "package://{}/srdf/{}.srdf".format(
                self._rm.pkg_name, self._rm.srdf_name
            )
            rootJointType = root_joint_type

            # \param compositeName name of the composite robot that will be built later,
            # \param robotName name of the first robot that is loaded now,
            # \param load whether to actually load urdf files. Set to false if you only
            #        want to initialize a corba client to an already initialized
            #        problem.
            # \param rootJointType type of root joint among ("freeflyer", "planar",
            #        "anchor"),
            def __init__(
                self,
                compositeName,
                robotName,
                load=True,
                rootJointType="planar",
                **kwargs
            ):
                Parent.__init__(
                    self, compositeName, robotName, rootJointType, load, **kwargs
                )

        robot = CompositeRobot(
            "{}-{}".format(self._rm.name, self._om.name), self._rm.name
        )
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
                self._gm.fingers[i],
                "{}/{}".format(self._rm.name, self._gm.joints[i]),
                [
                    self._gm.values[i],
                ],
            )
        return lock_hand

    def _create_constrain_graph(self):
        constrain_graph = ConstraintGraph(self._robot, "graph")
        factory = ConstraintGraphFactory(constrain_graph)
        factory.setGrippers(
            [
                "{}/{}".format(self._rm.name, self._gm.name),
            ]
        )
        factory.environmentContacts(
            [
                "{}/{}".format(self._em.name, self._em.surface),
            ]
        )
        factory.setObjects(
            [
                self._om.name,
            ],
            [
                [
                    "{}/{}".format(self._om.name, self._om.handle),
                ],
            ],
            [
                [
                    "{}/{}".format(self._om.name, self._om.surface),
                ],
            ],
        )
        factory.setRules(
            [
                Rule([".*"], [".*"], True),
            ]
        )
        factory.generate()
        constrain_graph.addConstraints(
            graph=True, constraints=Constraints(lockedJoints=self._lock_hand)
        )
        constrain_graph.initialize()
        return constrain_graph

    def update_current_config(self, msg):
        if isinstance(msg, JointState):
            for i, joint_name in enumerate(msg.name):
                try:
                    rank = self._robot.rankInConfiguration[
                        "{}/{}".format(self._robot_name, joint_name)
                    ]
                    self._q_current[rank] = msg.position
                except KeyError:
                    continue
        elif isinstance(msg, Odometry):
            self._set_robot_base_config(self._q_current, msg)
        elif isinstance(msg, Pose):
            self._set_object_config(self._q_current, msg)
        else:
            rospy.logerr(
                "Msg is not of type JointState/Odometry/Pose: {}".format(type(msg))
            )

    def set_object_goal_config(self, object_pose):
        assert isinstance(object_pose, Pose)
        self._set_object_config(self._q_goal, object_pose)

    def get_current_base_global_pose(self):
        return self._get_robot_base_pose(self._q_current)

    def make_plan(self, pos_tol, ori_tol):
        res, q_init_proj, err = self._constrain_graph.applyNodeConstraints(
            "free", self._q_current
        )
        res, q_goal_proj, err = self._constrain_graph.applyNodeConstraints(
            "free", self._q_goal
        )

        self._problem_solver.setInitialConfig(q_init_proj)
        self._problem_solver.addGoalConfig(q_goal_proj)
        self._problem_solver.solve()

        self._problem_solver.setTargetState(
            self._constrain_graph.nodes[
                "{}/{} grasps {}/{}".format(
                    self._rm.name, self._gm.name, self._om.name, self._om.handle
                )
            ]
        )
        self._problem_solver.solve()

        viewer = self._viewer_factory.createViewer()
        viewer(q_init_proj)
        path_player = PathPlayer(viewer)

        path_player(0)
        path_player(1)

    @staticmethod
    def _get_robot_base_pose(config):
        pose = Pose()
        pose.position.x = config[0]
        pose.position.y = config[1]
        cos_yaw = config[2]
        sin_yaw = config[3]
        yaw = math.atan2(sin_yaw, cos_yaw)
        pose.orientation = common.to_ros_orientation(transform.euler_matrix(0, 0, yaw))
        return pose

    @staticmethod
    def _set_robot_base_config(config, base_odom):
        assert isinstance(base_odom, Odometry)
        base_pose = base_odom.pose.pose
        config[0:2] = [base_pose.position.x, base_pose.position.y]
        yaw = transform.euler_from_matrix(common.sd_orientation(base_pose.orientation))[
            -1
        ]
        config[2:4] = [math.cos(yaw), math.sin(yaw)]

    def _set_object_config(self, config, object_pose):
        assert isinstance(object_pose, Pose)
        rank = self._robot.rankInConfiguration[
            "{}/root_joint".format(self._object_name)
        ]
        # TODO check the order is wxyz
        config[rank : rank + 7] = [
            object_pose.position.x,
            object_pose.position.y,
            object_pose.position.z,
            object_pose.orientation.w,
            object_pose.orientation.x,
            object_pose.orientation.y,
            object_pose.orientation.z,
        ]
