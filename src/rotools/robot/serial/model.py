"""Robot module."""
from typing import Sized

import numpy as np  # type: ignore
import scipy.optimize  # type: ignore

from attr import attrs, attrib, Factory

from rotools.robot.serial.kinematic_chain import MDHKinematicChain, POEKinematicChain
from rotools.robot.serial.tool import Tool

from rotools.utility.json_encoder import JSONEncoder
from rotools.utility import common, transform, robotics


def config_factory(robot):
    if robot.mdh is not None or robot.poe is not None:
        return np.zeros(robot.dof, dtype=float)
    else:
        raise ValueError("No MDH or POE param defined")


def config_limits_factory(robot):
    return np.repeat((-np.pi / 2, np.pi / 2), robot.dof).reshape((2, -1))


@attrs
class RobotModel(Sized):
    """Robot manipulator class."""

    tool = attrib(factory=lambda: Tool(), type=Tool)
    world_frame = attrib(factory=lambda: np.eye(4), type=np.ndarray)

    # Used in PoE calculation
    poe = attrib(default=None, type=POEKinematicChain)

    # Used in MDH calculation
    mdh = attrib(default=None, type=MDHKinematicChain)

    # For transforming eef pose in the arm's base frame to the robot's base frame or getting the inverse transform.
    arm_base_to_robot_base_trans = attrib(
        factory=lambda: transform.identity_matrix(), type=np.ndarray
    )

    random_state = attrib(
        factory=lambda: np.random.RandomState(),
        type=np.random.RandomState,
    )
    q0 = attrib(
        default=Factory(factory=config_factory, takes_self=True),
        type=np.ndarray,
    )
    _q = attrib(
        default=Factory(factory=config_factory, takes_self=True),
        type=np.ndarray,
    )
    _dq = attrib(
        default=Factory(factory=config_factory, takes_self=True),
        type=np.ndarray,
    )
    _q_limits = attrib(
        default=Factory(factory=config_limits_factory, takes_self=True),
        type=np.ndarray,
    )
    _dq_limits = attrib(
        default=Factory(factory=config_limits_factory, takes_self=True),
        type=np.ndarray,
    )

    def __len__(self):
        """Get the degrees of freedom.

        :return: degrees of freedom
        """
        if self.poe is not None:
            return len(self.poe)
        else:
            return len(self.mdh)

    def to_json(self):
        """Encode robot model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    def fk(self, q):
        """Compute the forward kinematics of a given joint state.

        :param q: Sequence[float]
        :return: 4x4 transform matrix of the FK pose
        """
        if self.poe is not None:
            pose = robotics.fk_in_space(self.poe.home_matrix, self.poe.screw_axes, q)
        else:
            transforms = [self.world_frame]
            transforms.extend(
                self.mdh.transforms(q)
            )  # Add n=dof transforms to the list
            transforms.append(self.tool.matrix)

            # matrix multiply through transforms
            pose = np.eye(4, dtype=float)
            for t in transforms:
                pose = np.dot(pose, t)
        return pose

    def partial_fk(self, q, i):
        if i >= self.dof:
            raise IndexError
        transforms = []
        transforms.extend(
            self.mdh.transforms(q[: i + 1])
        )  # Add n=dof transforms to the list

        # matrix multiply through transforms
        pose = np.eye(4, dtype=float)
        for t in transforms:
            pose = np.dot(pose, t)
        return pose

    def fk_to_base(self, q):
        """Given joint states, compute the fk relative to manually set base frame
        with arm_base_to_robot_base_trans. This function is useful when the robot arm's static frame is
        not the reference frame of the robot (like the scenario for humanoids)

        :param q: ndarray, the joint states of serial robots
        """
        pose_eef_in_arm_base = self.fk(q)
        return np.dot(self.arm_base_to_robot_base_trans, pose_eef_in_arm_base)

    def ik_in_space(self, target_pose, current_q):
        """Solve the inverse kinematics.

        Args:
            target_pose: ndarray (4, 4) Transformation matrix of the eef in the arm's base frame
            current_q: Sequence[float] The current joint state

        Returns:
            ndarray/None
        """
        target_pose = common.sd_pose(target_pose, check=True)
        if self.poe is not None:
            target_q, ok = robotics.ik_in_space(
                self.poe.screw_axes,
                self.poe.home_matrix,
                target_pose,
                current_q,
                1e-2,
                1e-3,
            )
            return target_q if ok else None

        result = scipy.optimize.least_squares(
            fun=_ik_cost_function,
            x0=current_q,
            bounds=self.q_limits,
            args=(self, target_pose),
        )

        if result.success:  # pragma: no cover
            actual_pose = self.fk(result.x)
            if np.allclose(actual_pose, target_pose, atol=1e-3):
                return result.x
        return None

    def mdh_to_poe(self):
        if self.mdh is None:
            return None

        home_matrix = self.fk(self.q0)
        screw_axes = []
        for i in range(self.dof):
            q = np.zeros(i + 1)
            trans_0 = self.partial_fk(q, i)
            q[-1] = np.pi / 2.0
            trans_1 = self.partial_fk(q, i)
            trans_rot = np.dot(np.linalg.inv(trans_0), trans_1)

            # This is the rotation axis of joint i in its frame, where 0, 1, 2 for x, y, z
            local_axis_i = 0
            for col in range(3):
                if trans_rot[col, col] == 1.0:
                    local_axis_i = col

            # Get the global axis corresponding to the local axis, which is by definition the omega component of S.
            global_axis_i = trans_0[:3, local_axis_i]
            origin_i = trans_0[:3, -1]
            vel_i = -np.cross(global_axis_i, origin_i)
            screw_axis = np.concatenate([global_axis_i, vel_i])
            screw_axes.append(screw_axis)

        screw_axes = np.asarray(screw_axes).T
        poe = POEKinematicChain.from_parameters([home_matrix, screw_axes])
        return poe

    @property
    def dof(self):
        """Get the number of degrees of freedom.

        :return: DoF
        """
        return len(self)

    @property
    def q(self):
        """Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._q

    @q.setter
    def q(self, value):
        """Set joints."""
        if np.any(value < self.q_limits[0]) or np.any(value > self.q_limits[1]):
            raise ValueError
        self._q = value

    @property
    def dq(self):
        """Get the joint velocities in rad/s."""
        return self._dq

    @dq.setter
    def dq(self, value):
        """Set joint velocities in rad/s with ndarray [dof,]

        :param value:
        :return:
        """
        if np.any(value < self.dq_limits[0]) or np.any(value > self.dq_limits[1]):
            raise ValueError(
                "Velocity to set {} out of permissible range".format(value)
            )
        self._dq = value

    @property
    def dq_limits(self):
        return self._dq_limits

    @dq_limits.setter
    def dq_limits(self, value):
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise ValueError(
                "Velocity limits should have the shape (2, dof), 2 for min max"
            )
        self._dq_limits = value

    @property
    def q_limits(self):
        """
        Limits of the robot position (e.g., joint limits).

        :return: limits with shape (2,num_dof) where first row is upper limits
        """
        return self._q_limits

    @q_limits.setter
    def q_limits(self, value):
        """Set joint limits."""
        assert isinstance(value, np.ndarray)
        if value.shape[0] == 2 and value.shape[1] == len(self):
            self._q_limits = value
        elif value.shape[0] == len(self) and value.shape[1] == 2:
            self._q_limits = value.T
        else:
            raise ValueError(
                "Joint limits should have the shape (2, dof), 2 for min max, but the given shape is {}".format(
                    value.shape
                )
            )

    def jacobian_space(self, q=None):
        """Calculate the Jacobian wrt the world frame."""
        q = self.q if q is None else q
        if self.poe is not None:
            return robotics.jacobian_space(self.poe.screw_axes, q)

        j_fl = self.jacobian_flange(q)
        pose = self.fk(q)
        rotation = pose[:3, :3]
        j_tr = np.zeros((6, 6), dtype=float)
        j_tr[:3, :3] = rotation
        j_tr[3:, 3:] = rotation
        j_w = np.dot(j_tr, j_fl)
        return j_w

    def jacobian_flange(self, q=None):
        """Calculate the Jacobian wrt the flange frame.

        Args:
            q:

        Returns:

        """
        q = self.q if q is None else q

        # Initialize the Cartesian jacobian matrix (6-dof in space)
        jacobian_flange = np.zeros((6, self.dof))
        current_trans = self.tool.matrix.copy()

        for i in reversed(range(self.dof)):
            d = np.array(
                [
                    -current_trans[0, 0] * current_trans[1, 3]
                    + current_trans[1, 0] * current_trans[0, 3],
                    -current_trans[0, 1] * current_trans[1, 3]
                    + current_trans[1, 1] * current_trans[0, 3],
                    -current_trans[0, 2] * current_trans[1, 3]
                    + current_trans[1, 2] * current_trans[0, 3],
                ]
            )
            delta = current_trans[2, 0:3]
            jacobian_flange[:, i] = np.hstack((d, delta))
            current_link = self.mdh.links[i]
            p = q[i]
            current_link_transform = current_link.transform(p)
            current_trans = np.dot(current_link_transform, current_trans)

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
            q = self.q

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
            t = self.mdh.links[i].transform(p)

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
        return np.clip(q, self.q_limits[0], self.q_limits[1])

    def random_valid_q(self):
        """Generate random joints within limits."""
        return self.random_state.uniform(low=self.q_limits[0], high=self.q_limits[1])

    @classmethod
    def get_model_from_mdh(cls, mdh, joint_limits=None):
        """Construct Robot from Kinematic Chain parameters."""
        mdh_model = MDHKinematicChain.from_parameters(mdh)
        model = cls(mdh=mdh_model)
        if joint_limits is not None:
            model.q_limits = joint_limits
        model.poe = model.mdh_to_poe()
        return model

    @classmethod
    def get_model_from_poe(cls, poe, joint_limits=None):
        """Construct robot model from product of exponential parameters."""
        poe_model = POEKinematicChain.from_parameters(poe)
        model = cls(poe=poe_model)
        if joint_limits is not None:
            model.q_limits = joint_limits
        return model


def _ik_cost_function(current_q, robot, target_pose):
    current_pose = robot.fk(current_q)
    diff = np.abs(current_pose - target_pose)
    return diff.ravel()
