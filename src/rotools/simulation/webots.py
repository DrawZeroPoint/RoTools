from __future__ import print_function

import time
import numpy as np

# Webots controller lib
from controller import Supervisor

from rotools.utility import common
from rotools.utility import kinematics


class WebotsInterfaceUR(object):
    """Interface for performing UR series robot simulation in Webots."""

    def __init__(self):
        super(WebotsInterfaceUR, self).__init__()

        self.robot = Supervisor()
        print('Robot name: ', self.robot.getName())

        # To get TCP pose, we need add a Transform to the children slot
        # of the last endPoint's children->Transform->children,
        # and change the DEF name of the Transform to UR10_TCP
        self.base_node = self.robot.getFromDef('UR_BASE')
        self.eef_node = self.robot.getFromDef('UR_TCP')

        self.time_step = int(self.robot.getBasicTimeStep())

        # modify this according to robot configuration
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.motors = []
        for name in self.joint_names:
            motor = self.robot.getMotor(name)
            motor.setVelocity(motor.getMaxVelocity() * 0.5)
            self.motors.append(motor)

        # the position sensors must be enabled before usage
        self._set_joint_position_sensor(enabled=True)

    def get_joint_state(self):
        """Get joint states of the robot

        :return: List[float]
        """
        joint_states = []
        while self.robot.step(self.time_step) != -1:
            for motor in self.motors:
                try:
                    js = motor.getPositionSensor().getValue()
                    joint_states.append(js)
                except NotImplementedError:
                    pass
            if joint_states and not np.isnan(joint_states[0]):
                break
        return joint_states

    def get_tcp_pose(self):
        """Get the eef (i.e., the TCP) pose as SE(3)
        in the robot's base frame.

        :return: 4x4 homogeneous transformation matrix
        """
        # w: world frame, s: robot base frame, b: tcp frame
        p_wb = np.array(self.eef_node.getPosition())
        R_wb = np.array(self.eef_node.getOrientation()).reshape((3, -1))
        T_wb = kinematics.rp_to_homo_trans(R_wb, p_wb)

        p_ws = np.array(self.base_node.getPosition())
        R_ws = np.array(self.base_node.getOrientation()).reshape((3, -1))
        T_ws = kinematics.rp_to_homo_trans(R_ws, p_ws)

        return np.dot(kinematics.homo_trans_inv(T_ws), T_wb)

    def go_to_joint_state(self, joint_goal, tolerance=1e-4):
        """Set the joint states as desired.

        :param joint_goal: List[float] joint state in rad
        :param tolerance: Max distance for positioning error
        :return:
        """
        while self.robot.step(self.time_step) != -1:
            for i, goal in enumerate(joint_goal):
                try:
                    self.motors[i].setPosition(goal)
                except ValueError:
                    pass

            joint_current = self.get_joint_state()
            if common.all_close(joint_goal, joint_current, tolerance):
                break

    def _set_joint_position_sensor(self, enabled=True):
        """Enable/disable position sensors of the joints.

        """
        for motor in self.motors:
            try:
                js_sensor = motor.getPositionSensor()
                js_sensor.enable(self.time_step)
            except NotImplementedError:
                pass

    def set_joint_max_velocity(self, velocity):
        """

        """
        if isinstance(velocity, list):
            for i, vel in enumerate(velocity):
                try:
                    self.motors[i].setVelocity(vel)
                except ValueError:
                    pass
        else:
            for motor in self.motors:
                try:
                    motor.setVelocity(velocity)
                except ValueError:
                    pass

