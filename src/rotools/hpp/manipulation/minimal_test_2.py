#!/usr/bin/env python
import math
import subprocess

import numpy as np
from collections import namedtuple

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


def rotation_matrix(angle, direction, point=None):
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.array(
        ((cosa, 0.0, 0.0), (0.0, cosa, 0.0), (0.0, 0.0, cosa)), dtype=np.float64
    )
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        (
            (0.0, -direction[2], direction[1]),
            (direction[2], 0.0, -direction[0]),
            (-direction[1], direction[0], 0.0),
        ),
        dtype=np.float64,
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float64, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M


def unit_vector(data, axis=None, out=None):
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data


class Object(object):
    rootJointType = "freeflyer"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, object_name, pkg_name):
        self.__setattr__("urdfName", object_name)
        self.__setattr__("packageName", pkg_name)
        self.__setattr__("meshPackageName", pkg_name)


class Environment(object):
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, env_name, pkg_name):
        self.__setattr__("urdfName", env_name)
        self.__setattr__("packageName", pkg_name)
        self.__setattr__("meshPackageName", pkg_name)


class Model(object):
    def __init__(
        self, name, pkg_name, urdf_name="", srdf_name="", surface="", handle=""
    ):
        self._name = name
        self._pkg_name = pkg_name
        if urdf_name == "":
            self._urdf_name = name
        else:
            self._urdf_name = urdf_name
        if srdf_name == "":
            self._srdf_name = name
        else:
            self._srdf_name = srdf_name
        self._surface = surface
        self._handle = handle

    @property
    def name(self):
        return self._name

    @property
    def pkg_name(self):
        return self._pkg_name

    @property
    def urdf_name(self):
        return self._urdf_name

    @property
    def srdf_name(self):
        return self._srdf_name

    @property
    def surface(self):
        return self._surface

    @property
    def handle(self):
        return self._handle


class Gripper(object):
    def __init__(self, robot_model, name, fingers, finger_joints, joint_values):
        super(Gripper, self).__init__()

        self._dof = 2
        assert len(fingers) == len(finger_joints) == len(joint_values) == self._dof

        self._fingers = fingers
        self._finger_joints = finger_joints
        self._joint_values = joint_values

        self._name = name
        self._robot_model = robot_model

    @property
    def dof(self):
        return self._dof

    @property
    def name(self):
        return self._name

    @property
    def fingers(self):
        return self._fingers

    @property
    def joints(self):
        return self._finger_joints

    @property
    def values(self):
        return self._joint_values

    def get_urdf(self):
        pass


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

        self._work_modes = namedtuple("work_modes", "idle approach grasp")
        self._previous_mode = self._work_modes.idle
        self._mode = self._work_modes.idle

        self._robot = self._create_robot()
        self._problem_solver = ProblemSolver(self._robot)

        self._problem_solver.loadPlugin("manipulation-spline-gradient-based.so")
        self._problem_solver.addPathOptimizer("RandomShortcut")

        self._viewer_factory = (
            self._create_viewer_factory()
        )  # need to create this no matter if enable_viewer

        self._robot.setJointBounds("{}/root_joint".format(self._rm.name), robot_bound)
        self._robot.setJointBounds("{}/root_joint".format(self._om.name), object_bound)

        # An absolute value, if the threshold is surpassed, will raise the error `A configuration has no node`
        self._problem_solver.setErrorThreshold(1e-3)
        self._problem_solver.setMaxIterProjection(80)

        self._problem_solver.setTimeOutPathPlanning(30)

        print(
            "Using path planner {}, available planners are: {}".format(
                self._problem_solver.getSelected("PathPlanner"),
                self._problem_solver.getAvailable("PathPlanner"),
            )
        )

        self._q_init = self._robot.getCurrentConfig()
        self._q_goal = self._q_init[::]

        self._lock_hand = self._create_locks()

        self._graph_id = 0
        self._constraint_graph = self._create_constraint_graph()

        self._object_transform = rotation_matrix(-np.pi / 2, (0, 1, 0))

    def _create_robot(self, root_joint_type="planar"):
        class CompositeRobot(Parent):
            urdfFilename = "package://{}/urdf/{}.urdf".format(
                self._rm.pkg_name, self._rm.urdf_name
            )
            srdfFilename = "package://{}/srdf/{}.srdf".format(
                self._rm.pkg_name, self._rm.srdf_name
            )
            rootJointType = root_joint_type

            def __init__(
                self,
                composite_name,
                robot_name,
                load=True,
                root_joint="planar",
                **kwargs
            ):
                Parent.__init__(
                    self, composite_name, robot_name, root_joint, load, **kwargs
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

    def _create_locks(self):
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

    def _create_constraint_graph(self):
        self._graph_id += 1
        graph_name = "graph_{}".format(self._graph_id)
        constraint_graph = ConstraintGraph(self._robot, graph_name)
        rospy.loginfo("Created new constraint graph: {}".format(graph_name))
        factory = ConstraintGraphFactory(constraint_graph)
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
        constraint_graph.addConstraints(
            graph=True, constraints=Constraints(numConstraints=self._lock_hand)
        )
        constraint_graph.initialize()
        return constraint_graph

    def _make_plan(self):
        self._problem_solver.addGoalConfig(self._q_goal)
        self._problem_solver.setInitialConfig(self._q_init)

        print(
            "Current configuration:\n{}".format(
                ["{0:0.8f}".format(i) for i in self._q_init]
            )
        )
        print(
            "Goal configuration:\n{}".format(
                ["{0:0.8f}".format(i) for i in self._q_goal]
            )
        )

        try:
            time_spent = self._problem_solver.solve()
            print("Plan solved in {}h-{}m-{}s-{}ms".format(*time_spent))
        except BaseException as e:
            print(self._constraint_graph.nodes)
            print("Failed to solve due to {}".format(e))
            res, q, err = self._constraint_graph.applyNodeConstraints(
                "free", self._q_init
            )
            print(res, err, "\n", q)
            print(self._constraint_graph.getNode(self._q_init))
            self._problem_solver.resetGoalConfigs()
            self._mode = self._work_modes.idle
            return False

        viewer = self._viewer_factory.createViewer()
        viewer(self._q_init)
        path_player = PathPlayer(viewer)
        while (
            input(
                "Plan finished! Enter q to quit the current path, enter other keys to replay\n"
            )
            != "q"
        ):
            path_player(self._last_path_id)

        self._problem_solver.resetGoalConfigs()
        self._mode = self._work_modes.idle
        return True

    def make_test_plan(self):
        """Make a grasping plan.

        Returns:
            True if success, False otherwise.
        """
        # [0:4] position and orientation of the base in the world frame;
        # [4:7] 2 head joints and one torso joint;
        # [7:14] left arm joints
        # [14:16] left gripper joints
        # [16:23] right arm joints
        # [23:25] right gripper joints
        # [25:32] object pose
        # FIXME This works
        # self._q_current = [-1.20326676, -0.59723202, -0.99997469, -0.00711415, -0.00000287, 0.00000000, 0.00023268,
        #                    1.00024558, 0.80147044, -0.49938611, -1.98775687, 0.00035514, 2.50160673, 1.57078452,
        #                    0.03928489, 0.04000000, -1.00150619, 0.80181510, 0.49849963, -1.98774401, 0.00041852,
        #                    2.50161359, -0.00001253, 0.04000000, 0.03928675, -2.40000000, -0.60000000, 0.96449543,
        #                    0.00000000, -0.70710678, 0.00000000, 0.70710678]
        # self._q_goal = [-1.20326676, -0.59723202, -0.99997469, -0.00711415, -0.00000287, 0.00000000, 0.00023268,
        #                 1.00024558, 0.80147044, -0.49938611, -1.98775687, 0.00035514, 2.50160673, 1.57078452,
        #                 0.03928489, 0.04000000, -1.00150619, 0.80181510, 0.49849963, -1.98774401, 0.00041852,
        #                 2.50161359, -0.00001253, 0.04000000, 0.03928675, -2.40000000, -0.65000000, 0.96500000,
        #                 0.00000000, -0.70710678, 0.00000000, 0.70710678]

        # FIXME This does not work
        self._q_init = [
            0.6828121022644879634,
            0.0703259613989929006,
            0.8687425541219662151,
            0.4952639444353310050,
            0.0001423814094132478,
            0.0000010956036508039,
            0.0003569450372355056,
            1.0006461567295650283,
            0.8029290429009514618,
            -0.4976525619349045360,
            -1.9705628949627718161,
            0.0007762117476604057,
            2.5024607950674822021,
            1.5698918488567741303,
            0.04,
            0.04,
            -1.0032131005843512028,
            0.8043047023383176342,
            0.4956911330734958265,
            -1.9707136657764974252,
            0.0008677962229538790,
            2.5024503019683517202,
            -0.0000280178770064655,
            0.04,
            0.04,
            -2.3999999994155452043,
            -0.5999999998769479648,
            0.9644954319622398575,
            0.0000000000000003189,
            -0.7071067811865474617,
            -0.0000000000000003182,
            0.7071067811865474617,
        ]
        self._q_goal = [
            0.97013911,
            0.21839769,
            0.70710678,
            0.70710678,
            0.00011383,
            0.00000100,
            0.00027770,
            1.00063293,
            0.80204085,
            -0.49794376,
            -1.97783062,
            0.00056355,
            2.50151300,
            1.56990079,
            0.03734613,
            0.04000000,
            -1.00249611,
            0.80319770,
            0.49647833,
            -1.97797381,
            0.00062338,
            2.50150087,
            -0.00001964,
            0.04000000,
            0.03622407,
            -2.40000000,
            -0.60000000,
            0.96449543,
            0.00000000,
            -0.70710678,
            -0.00000000,
            0.70710678,
        ]

        self._mode = self._work_modes.approach
        return self._make_plan()

    @property
    def _last_path_id(self):
        return self._problem_solver.numberPaths() - 1


if __name__ == "__main__":
    server_process = subprocess.Popen(["hppcorbaserver"], start_new_session=True)
    rospy.sleep(5)
    viewer_process = subprocess.Popen(
        ["gepetto-gui", "-c", "basic"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    try:
        interface = HPPManipulationInterface(
            "clover_lab",
            "clover_lab",
            "base_cabinet_worktop_top",
            "cube_30",
            "clover_lab",
            "cube_30_surface",
            "cube_30_handle2",
            "curi",
            "curi_description",
            "curi_pinocchio",
            "curi_pinocchio",
            [-2.5, 2.5, -1.5, 1.5],
            [-3, 3, -1.5, 1.5, 0, 2],
            "l_gripper",
            ["panda_left_leftfinger", "panda_left_rightfinger"],
            ["panda_left_finger_joint1", "panda_left_finger_joint2"],
            [0.04, 0.04],
        )
        interface.make_test_plan()
    except BaseException as e:
        print(e)
    if server_process is not None:
        server_process.kill()
    if viewer_process is not None:
        viewer_process.kill()
