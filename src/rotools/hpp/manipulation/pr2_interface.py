from __future__ import print_function

import math

import rospy

from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule, \
    Constraints, ConstraintGraphFactory, Client
from hpp.corbaserver import loadServerPlugin

from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from rotools.utility import common, transform


class Object(object):
    rootJointType = 'freeflyer'
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, object_name, pkg_name):
        self.__setattr__('urdfName', object_name)
        self.__setattr__('packageName', pkg_name)
        self.__setattr__('meshPackageName', pkg_name)


class Environment(object):
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, env_name, pkg_name):
        self.__setattr__('urdfName', env_name)
        self.__setattr__('packageName', pkg_name)
        self.__setattr__('meshPackageName', pkg_name)


class HPPManipulationInterface(object):

    def __init__(
            self,
            env_name,
            env_pkg_name,
            object_name,
            object_pkg_name,
            robot_name,
            robot_pkg_name,
            robot_urdf_name,
            robot_srdf_name,
            robot_bound=[-5, -2, -5.2, -2.7],
            object_bound=[-5.1, -2, -5.2, -2.7, 0, 1.5],
            **kwargs
    ):
        super(HPPManipulationInterface, self).__init__()

        loadServerPlugin("corbaserver", "manipulation-corba.so")
        Client().problem.resetProblem()

        self._robot_name = robot_name
        self._object_name = object_name

        self._robot = self._create_robot(robot_pkg_name, robot_urdf_name, robot_srdf_name)
        self._problem_solver = ProblemSolver(self._robot)

        self._viewer_factory = self._create_viewer_factory(object_pkg_name, env_name, env_pkg_name)

        self._robot.setJointBounds("{}/root_joint".format(robot_name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(object_name), object_bound)

        # robot.client.basic.problem.resetRoadmap ()
        self._problem_solver.setErrorThreshold(1e-3)
        self._problem_solver.setMaxIterProjection(40)

        # Generate initial and goal configuration.
        self._q_current = self._robot.getCurrentConfig()

        self._q_current[0:2] = [-3.2, -4]
        rank = self._robot.rankInConfiguration['{}/l_gripper_l_finger_joint'.format(robot_name)]
        self._q_current[rank] = 0.5
        rank = self._robot.rankInConfiguration['{}/l_gripper_r_finger_joint'.format(robot_name)]
        self._q_current[rank] = 0.5
        rank = self._robot.rankInConfiguration['{}/torso_lift_joint'.format(robot_name)]
        self._q_current[rank] = 0.2

        rank = self._robot.rankInConfiguration['{}/root_joint'.format(object_name)]
        self._q_current[rank:rank + 3] = [-2.5, -4, 0.8]

        self._q_goal = self._q_current[::]
        self._q_goal[0:2] = [-3.2, -4]
        rank = self._robot.rankInConfiguration['{}/root_joint'.format(object_name)]
        self._q_goal[rank:rank + 3] = [-3.5, -4, 0.8]

        locklhand = ['l_l_finger', 'l_r_finger']
        self._problem_solver.createLockedJoint('l_l_finger', '{}/l_gripper_l_finger_joint'.format(robot_name), [0.5, ])
        self._problem_solver.createLockedJoint('l_r_finger', '{}/l_gripper_r_finger_joint'.format(robot_name), [0.5, ])

        self._constrain_graph = ConstraintGraph(self._robot, 'graph')
        factory = ConstraintGraphFactory(self._constrain_graph)
        factory.setGrippers(["{}/l_gripper".format(robot_name), ])
        factory.environmentContacts(["{}/pancake_table_table_top".format(env_name), ])
        factory.setObjects([object_name, ],
                           [["{}/handle2".format(object_name), ], ],
                           [["{}/box_surface".format(object_name), ], ])
        factory.setRules([Rule([".*"], [".*"], True), ])
        factory.generate()
        self._constrain_graph.addConstraints(graph=True, constraints=Constraints(lockedJoints=locklhand))
        self._constrain_graph.initialize()

    def _create_robot(self, pkg_name, urdf_name, srdf_name, root_joint_type='planar'):
        class CompositeRobot(Parent):
            urdfFilename = "package://{}/urdf/{}.urdf".format(pkg_name, urdf_name)
            srdfFilename = "package://{}/srdf/{}.srdf".format(pkg_name, srdf_name)
            rootJointType = root_joint_type

            # \param compositeName name of the composite robot that will be built later,
            # \param robotName name of the first robot that is loaded now,
            # \param load whether to actually load urdf files. Set to no if you only
            #        want to initialize a corba client to an already initialized
            #        problem.
            # \param rootJointType type of root joint among ("freeflyer", "planar",
            #        "anchor"),
            def __init__(self, compositeName, robotName, load=True,
                         rootJointType="planar", **kwargs):
                Parent.__init__(self, compositeName, robotName, rootJointType, load, **kwargs)

        robot = CompositeRobot('{}-{}'.format(self._robot_name, self._object_name), self._robot_name)
        return robot

    def _create_viewer_factory(self, object_pkg_name, env_name, env_pkg_name):
        object_to_grasp = Object(self._object_name, object_pkg_name)
        environment = Environment(env_name, env_pkg_name)
        viewer_factory = ViewerFactory(self._problem_solver)
        viewer_factory.loadObjectModel(object_to_grasp, self._object_name)
        viewer_factory.loadEnvironmentModel(environment, env_name)
        return viewer_factory

    def update_current_config(self, msg):
        if isinstance(msg, JointState):
            for i, joint_name in enumerate(msg.name):
                try:
                    rank = self._robot.rankInConfiguration['{}/{}'.format(self._robot_name, joint_name)]
                    self._q_current[rank] = msg.position
                except KeyError:
                    continue
        elif isinstance(msg, Odometry):
            self._set_robot_base_config(self._q_current, msg)
        elif isinstance(msg, Pose):
            self._set_object_config(self._q_current, msg)
        else:
            rospy.logerr("Msg is not of type JointState/Odometry/Pose: {}".format(type(msg)))

    def set_object_goal_config(self, object_pose):
        assert isinstance(object_pose, Pose)
        self._set_object_config(self._q_goal, object_pose)

    def get_current_base_global_pose(self):
        return self._get_robot_base_pose(self._q_current)

    def make_plan(self):
        res, q_init_proj, err = self._constrain_graph.applyNodeConstraints("free", self._q_current)
        res, q_goal_proj, err = self._constrain_graph.applyNodeConstraints("free", self._q_goal)

        self._problem_solver.setInitialConfig(q_init_proj)
        self._problem_solver.addGoalConfig(q_goal_proj)
        self._problem_solver.solve()

        self._problem_solver.setTargetState(
            self._constrain_graph.nodes["{}/l_gripper grasps {}/handle2".format(self._robot_name, self._object_name)])
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
        yaw = transform.euler_from_matrix(common.sd_orientation(base_pose.orientation))[-1]
        config[2:4] = [math.cos(yaw), math.sin(yaw)]

    def _set_object_config(self, config, object_pose):
        assert isinstance(object_pose, Pose)
        rank = self._robot.rankInConfiguration['{}/root_joint'.format(self._object_name)]
        # TODO check the order is wxyz
        config[rank: rank + 7] = [object_pose.position.x, object_pose.position.y, object_pose.position.z,
                                  object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y,
                                  object_pose.orientation.z]
