from __future__ import print_function

import os
import time
import rospy

from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule, \
    Constraints, ConstraintGraphFactory, Client
from hpp.corbaserver import loadServerPlugin

from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState, Image, CompressedImage
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

        self._robot = self._create_robot(object_name, robot_name, robot_pkg_name, robot_urdf_name, robot_srdf_name)
        self._problem_solver = ProblemSolver(self._robot)

        self._viewer_factory = self._create_viewer_factory(object_name, object_pkg_name, env_name, env_pkg_name)

        self._robot.setJointBounds("{}/root_joint".format(robot_name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(object_name), object_bound)

        # robot.client.basic.problem.resetRoadmap ()
        self._problem_solver.setErrorThreshold(1e-3)
        self._problem_solver.setMaxIterProjection(40)

        # Generate initial and goal configuration.
        q_init = self._robot.getCurrentConfig()
        q_init[0:2] = [-3.2, -4]
        rank = self._robot.rankInConfiguration['{}/l_gripper_l_finger_joint'.format(robot_name)]
        q_init[rank] = 0.5
        rank = self._robot.rankInConfiguration['{}/l_gripper_r_finger_joint'.format(robot_name)]
        q_init[rank] = 0.5
        rank = self._robot.rankInConfiguration['{}/torso_lift_joint'.format(robot_name)]
        q_init[rank] = 0.2

        rank = self._robot.rankInConfiguration['{}/root_joint'.format(object_name)]
        q_init[rank:rank + 3] = [-2.5, -4, 0.8]

        q_goal = q_init[::]
        q_goal[0:2] = [-3.2, -4]
        rank = self._robot.rankInConfiguration['{}/root_joint'.format(object_name)]
        q_goal[rank:rank + 3] = [-3.5, -4, 0.8]

        locklhand = ['l_l_finger', 'l_r_finger']
        self._problem_solver.createLockedJoint('l_l_finger', '{}/l_gripper_l_finger_joint'.format(robot_name), [0.5, ])
        self._problem_solver.createLockedJoint('l_r_finger', '{}/l_gripper_r_finger_joint'.format(robot_name), [0.5, ])

        cg = ConstraintGraph(self._robot, 'graph')
        factory = ConstraintGraphFactory(cg)
        factory.setGrippers(["{}/l_gripper".format(robot_name), ])
        factory.environmentContacts(["{}/pancake_table_table_top".format(env_name), ])
        factory.setObjects([object_name, ],
                           [["{}/handle2".format(object_name), ], ],
                           [["{}/box_surface".format(object_name), ], ])
        factory.setRules([Rule([".*"], [".*"], True), ])
        factory.generate()
        cg.addConstraints(graph=True, constraints=Constraints(lockedJoints=locklhand))
        cg.initialize()

        res, q_init_proj, err = cg.applyNodeConstraints("free", q_init)
        res, q_goal_proj, err = cg.applyNodeConstraints("free", q_goal)

        self._problem_solver.setInitialConfig(q_init_proj)
        self._problem_solver.addGoalConfig(q_goal_proj)
        self._problem_solver.solve()

        self._problem_solver.setTargetState(cg.nodes["{}/l_gripper grasps {}/handle2".format(robot_name, object_name)])
        self._problem_solver.solve()

        viewer = self._viewer_factory.createViewer()
        viewer(q_init_proj)
        path_player = PathPlayer(viewer)

        path_player(0)
        path_player(1)

    def _create_robot(self, object_name, robot_name, pkg_name, urdf_name, srdf_name, root_joint_type='planar'):
        class CompositeRobot(Parent):
            urdfFilename = "package://{}/urdf/{}.urdf".format(pkg_name, urdf_name)
            srdfFilename = "package://{}/srdf/{}.srdf".format(pkg_name, srdf_name)
            rootJointType = root_joint_type

            ## Constructor
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

        robot = CompositeRobot('{}-{}'.format(robot_name, object_name), robot_name)
        return robot

    def _create_viewer_factory(self, object_name, object_pkg_name, env_name, env_pkg_name):
        object_to_grasp = Object(object_name, object_pkg_name)
        environment = Environment(env_name, env_pkg_name)
        viewer_factory = ViewerFactory(self._problem_solver)
        viewer_factory.loadObjectModel(object_to_grasp, object_name)
        viewer_factory.loadEnvironmentModel(environment, env_name)
        return viewer_factory