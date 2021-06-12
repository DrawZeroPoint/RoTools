"""Robot module."""
from typing import Sized

import numpy as np  # type: ignore
import scipy.optimize  # type: ignore

from attr import attrs, attrib, Factory
from json_encoder import JSONEncoder
from _kinematic_chain import MDHKinematicChain
from _tool import Tool

from rotools.utility import common
from rotools.utility import transform
from rotools.utility import kinematics


def _ndof_zeros_factory(robot):
    if robot.kinematic_chain:
        dof = len(robot.kinematic_chain)
    else:
        dof = robot.screw_axes.shape[0]
    return np.zeros(dof)


def _joint_velocities_factory(robot):
    if robot.kinematic_chain:
        dof = len(robot.kinematic_chain)
    else:
        dof = robot.screw_axes.shape[0]
    return np.zeros(dof)


def _joint_vel_limits_factory(robot):
    if robot.kinematic_chain:
        dof = len(robot.kinematic_chain)
    else:
        dof = robot.screw_axes.shape[0]
    return np.repeat((-np.pi/2, np.pi/2), dof).reshape((2, -1))


def _joint_limits_factory(robot):
    if robot.kinematic_chain:
        dof = len(robot.kinematic_chain)
    else:
        dof = robot.screw_axes.shape[0]
    return np.repeat((-2*np.pi, 2*np.pi), dof).reshape((2, -1))


@attrs
class RobotModel(Sized):
    """Robot manipulator class."""

    tool = attrib(factory=lambda: Tool(), type=Tool)
    world_frame = attrib(factory=lambda: np.eye(4), type=np.ndarray)

    # Used in PoE calculation
    home_matrix = attrib(default=None)
    screw_axes = attrib(default=None)

    # Used in MDH calculation
    kinematic_chain = attrib(default=None, type=MDHKinematicChain)

    # For transform eef pose in arm base link to robot base link or inverse
    base_to_static_trans = attrib(factory=lambda: transform.identity_matrix(), type=np.ndarray)

    random_state = attrib(
        factory=lambda: np.random.RandomState(),
        type=np.random.RandomState,
    )
    home_position = attrib(
        default=Factory(factory=_ndof_zeros_factory, takes_self=True),
        type=np.ndarray,
    )
    _joints = attrib(
        default=Factory(factory=_ndof_zeros_factory, takes_self=True),
        type=np.ndarray,
    )
    _joint_velocities = attrib(
        default=Factory(factory=_joint_velocities_factory, takes_self=True),
        type=np.ndarray,
    )
    _vel_limits = attrib(
        default=Factory(factory=_joint_vel_limits_factory, takes_self=True),
        type=np.ndarray,
    )
    _joint_limits = attrib(
        default=Factory(factory=_joint_limits_factory, takes_self=True),
        type=np.ndarray,
    )

    def __len__(self):
        """Get the degrees of freedom.

        :return: degrees of freedom
        """
        if self.screw_axes is not None:
            return self.screw_axes.shape[0]
        else:
            return len(self.kinematic_chain)

    def to_json(self):
        """Encode robot model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    def fk(self, q):
        """Compute the forward kinematics of a given joint state.

        :param q: Sequence[float]
        :return: 4x4 transform matrix of the FK pose
        """
        if self.home_matrix is not None:
            pose = kinematics.fk_in_space(self.home_matrix, self.screw_axes, q)
        else:
            transforms = []
            transforms.append(self.world_frame)
            transforms.extend(self.kinematic_chain.transforms(q))
            transforms.append(self.tool.matrix)

            # matrix multiply through transforms
            pose = np.eye(4, dtype=float)
            for t in transforms:
                pose = np.dot(pose, t)
        return pose

    def fk_to_base(self, q):
        """Given joint states, compute the fk relative to manually set base frame
        with base_to_static_trans. This function is useful when the robot arm's static frame is
        not the reference frame of the robot (like the scenario for humanoids)

        :param q: ndarray, the joint states of serial robots
        """
        pose_eef_in_arm_base = self.fk(q)
        return np.dot(self.base_to_static_trans, pose_eef_in_arm_base)

    def ik(self, pose, q):
        """Solve the inverse kinematics.

        :param pose: 4x4 transformation matrix of the eef in the arm base frame
        :param q: Sequence[float] the current joint state
        :return ndarray or None
        """
        pose = common.sd_pose(pose)
        self.joints = q
        result = scipy.optimize.least_squares(fun=_ik_cost_function, x0=q,
                                              bounds=self.joint_limits, args=(pose, self))

        if result.success:  # pragma: no cover
            actual_pose = self.fk(result.x)
            if np.allclose(actual_pose, pose, atol=1e-3):
                return result.x
        return None

    @property
    def dof(self):
        """Get the number of degrees of freedom.

        :return: DoF
        """
        return len(self)

    @property
    def joints(self):
        """Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._joints

    @joints.setter
    def joints(self, value):
        """Set joints."""
        if np.any(value < self.joint_limits[0]) or np.any(value > self.joint_limits[1]):
            raise ValueError
        self._joints = value

    @property
    def joint_velocities(self):
        """Get the joint velocities in rad/s."""
        return self._joint_velocities

    @joint_velocities.setter
    def joint_velocities(self, value):
        """Set joint velocities in rad/s with ndarray [dof,]

        :param value:
        :return:
        """
        if np.any(value < self._vel_limits[0]) or np.any(value > self._vel_limits[1]):
            raise ValueError('Velocity to set {} out of permissible range'.format(value))
        self._joint_velocities = value

    @property
    def velocity_limits(self):
        return self._vel_limits

    @velocity_limits.setter
    def velocity_limits(self, value):
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise ValueError('Velocity limits should have the shape (2, dof), 2 for min max')
        self._vel_limits = value

    @property
    def joint_limits(self):
        """
        Limits of the robot position (e.g., joint limits).

        :return: limits with shape (2,num_dof) where first row is upper limits
        """
        return self._joint_limits

    @joint_limits.setter
    def joint_limits(self, value):
        """Set joint limits."""
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise ValueError('Joint limits should have the shape (2, dof), 2 for min max')
        self._joint_limits = value

    def jacobian_world(self, q=None):
        """Calculate the Jacobian wrt the world frame."""
        q = self.joints if q is None else q
        j_fl = self.jacobian_flange(q)
        pose = self.fk(q)
        rotation = pose[:3, :3]
        j_tr = np.zeros((6, 6), dtype=float)
        j_tr[:3, :3] = rotation
        j_tr[3:, 3:] = rotation
        j_w = np.dot(j_tr, j_fl)
        return j_w

    def jacobian_flange(self, q=None):
        """Calculate the Jacobian wrt the flange frame."""
        q = self.joints if q is None else q

        # init Cartesian jacobian (6-dof in space)
        jacobian_flange = np.zeros((6, self.dof))
        current_transform = self.tool.matrix.copy()

        for i in reversed(range(self.dof)):
            d = np.array(
                [
                    -current_transform[0, 0] * current_transform[1, 3]
                    + current_transform[1, 0] * current_transform[0, 3],
                    -current_transform[0, 1] * current_transform[1, 3]
                    + current_transform[1, 1] * current_transform[0, 3],
                    -current_transform[0, 2] * current_transform[1, 3]
                    + current_transform[1, 2] * current_transform[0, 3],
                ]
            )
            delta = current_transform[2, 0:3]
            jacobian_flange[:, i] = np.hstack((d, delta))
            current_link = self.kinematic_chain.links[i]
            p = q[i]
            current_link_transform = current_link.transform(p)
            current_transform = np.dot(current_link_transform, current_transform)

        return jacobian_flange

    def compute_joint_torques(self, wrench, q=None):
        """
        Calculate the joint torques due to external flange force.

        Method from:
        5.9 STATIC FORCES IN MANIPULATORS
        Craig, John J. Introduction to robotics: mechanics and control.
        Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
        :param wrench:
        :param q:
        :return:
        """
        if q is None:
            q = self.joints

        # split wrench into components
        force = wrench[:3]
        moment = wrench[-3:]

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base
        # each iteration calculates for link i-1
        for i, p in reversed(list(enumerate(q))):  # pragma: no cover
            if i == 0:
                break

            # get current link transform
            t = self.kinematic_chain.links[i].transform(p)

            # calculate force applied to current link
            rotation = t[:3, :3]
            force = np.dot(rotation, force)

            # calculate moment applied to current link
            q = t[:3, -1]
            moment = np.dot(rotation, moment) + np.cross(q, force)

            # append z-component as joint torque
            joint_torques.append(moment[-1])

        # reverse torques into correct order
        return np.array(list(reversed(joint_torques)), dtype=float)

    def clamp_joints(self, q):
        """Limit joints to joint limits."""
        return np.clip(q, self.joint_limits[0], self.joint_limits[1])

    def random_joints(self, in_place):
        """Generate random joints within limits."""
        q = self.random_state.uniform(
            low=self.joint_limits[0], high=self.joint_limits[1]
        )
        if in_place:
            self.joints = q
            return None
        else:
            return q

    @classmethod
    def from_mdh_parameters(cls, parameters):
        """Construct Robot from Kinematic Chain parameters."""
        # FIXME: assumes MDH revolute robot
        kc = MDHKinematicChain.from_parameters(parameters)
        return cls(kinematic_chain=kc)

    @classmethod
    def from_poe_parameters(cls, parameters):
        """Construct robot model from product of exponentials parameters."""
        if len(parameters) == 3:
            base_trans = parameters[2]
        else:
            base_trans = transform.identity_matrix()
        return cls(home_matrix=parameters[0], screw_axes=parameters[1], base_to_static_trans=base_trans)


def _ik_cost_function(q, pose, robot):
    actual_pose = robot.fk(q)
    diff = np.abs(actual_pose - pose)
    return diff.ravel()
