from __future__ import print_function

import os
import numpy as np
import xml.etree.ElementTree as ElementTree

from threading import Thread
from mujoco_py import MjSim, MjViewer, load_model_from_path

from rotools.utility.mjcf import find_elements, array_to_string

try:
    import rospy
    from sensor_msgs.msg import JointState
except ImportError:
    rospy = None
    JointState = None


class MuJoCoInterface(Thread):

    def __init__(
            self,
            model_path,
            actuator_path,
            enable_viewer=True,
            **kwargs
    ):
        super(MuJoCoInterface, self).__init__()
        Thread.__init__(self)

        if not os.path.exists(model_path):
            raise FileNotFoundError("Model XML file '{}' does not exist".format(model_path))

        self.model = load_model_from_path(model_path)
        self.sim = MjSim(self.model)

        if not os.path.exists(actuator_path):
            raise FileNotFoundError("Actuator XML file '{}' does not exist".format(actuator_path))

        tree = ElementTree.parse(actuator_path)
        root = tree.getroot()
        position_actuators = find_elements(root, 'position', return_first=False)
        velocity_actuators = find_elements(root, 'velocity', return_first=False)
        torque_actuators = find_elements(root, 'motor', return_first=False)

        """
        control_types: list[int] How the actuator is controlled.
                       The value could be: position (0), velocity (1), torque (2).
        """
        self.joint_names = []
        self.actuator_names = []
        self.control_types = []
        self._get_actuator_info(position_actuators, 0)
        self._get_actuator_info(velocity_actuators, 1)
        self._get_actuator_info(torque_actuators, 2)

        self.joint_num = len(self.joint_names)
        self.actuator_num = len(self.actuator_names)
        self.actuator_ids = [self.sim.model.actuator_name2id(actuator_name) for actuator_name in self.actuator_names]

        rospy.loginfo('Controlled joints #{}:\n{}'.format(self.joint_num, array_to_string(self.joint_names)))
        rospy.loginfo('Control types:\n{}'.format(array_to_string(self.control_types)))

        self.viewer = MjViewer(self.sim) if enable_viewer else None

        self._robot_states = None

    def run(self):
        while not rospy.is_shutdown():
            self._get_robot_states()
            self.sim.step()
            if self.viewer is not None:
                self.viewer.render()

    def _get_actuator_info(self, actuators, control_type):
        if actuators is not None:
            for actuator in actuators:
                self.joint_names.append(actuator.attrib['joint'])
                self.actuator_names.append(actuator.attrib['name'])
                self.control_types.append(control_type)

    def _get_robot_states(self):
        """Get joint qpos and qvel in each step.

        Returns:
            [qpos, qvel] for each joint given by the name.
        """
        robot_states = []
        sim_state = self.sim.get_state()
        for joint_name in self.joint_names:
            joint_qpos_id = self.sim.model.get_joint_qpos_addr(joint_name)
            joint_qpos = sim_state.qpos[joint_qpos_id]
            joint_qvel_id = self.sim.model.get_joint_qvel_addr(joint_name)
            joint_qvel = sim_state.qvel[joint_qvel_id]
            joint_state = [joint_qpos, joint_qvel]
            robot_states.append(joint_state)
        self._robot_states = np.array(robot_states)

    def get_joint_states(self):
        """Convert the robot_states to ROS JointState message.

        Returns:
            None if the robot_state is not available, otherwise return JointState
        """
        if self._robot_states is None:
            return None
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self._robot_states[:, 0].tolist()
        joint_state_msg.velocity = self._robot_states[:, 1].tolist()
        return joint_state_msg

    def set_joint_commands(self, cmd):
        """Obtain joint commands according to the internally defined control types and apply the command to the robot.

        Args:
            cmd: JointState Joint command message. It's the user's responsibility to match the command value's type
                 with the actuator's type.
        Returns:
            None
        """
        for i, actuator_id in enumerate(self.actuator_ids):
            if self.control_types[i] == 0:
                self.sim.data.ctrl[actuator_id] = cmd.position[i]
            elif self.control_types[i] == 1:
                self.sim.data.ctrl[actuator_id] = cmd.velocity[i]
            elif self.control_types[i] == 2:
                self.sim.data.ctrl[actuator_id] = cmd.effort[i]
            else:
                raise TypeError('Unsupported type {}'.format(self.control_types[i]))
