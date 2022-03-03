from __future__ import print_function

import os

from threading import Thread
from mujoco_py import MjSim, MjViewer, load_model_from_path

try:
    import rospy
    import tf2_ros
    import tf2_geometry_msgs  # import this is mandatory to use PoseStamped msg
    from sensor_msgs.msg import JointState
except ImportError:
    pass


class MuJoCoInterface(Thread):

    def __init__(
            self,
            model_path,
            joint_names,
            actuator_names,
            enable_viewer=True,
            **kwargs
    ):
        super(MuJoCoInterface, self).__init__()
        Thread.__init__(self)

        if not os.path.exists(model_path):
            raise FileNotFoundError("XML file '{}' does not exist".format(model_path))

        self.model = load_model_from_path(model_path)
        self.sim = MjSim(self.model)

        if enable_viewer:
            self.viewer = MjViewer(self.sim)

        if not isinstance(joint_names, list) and not isinstance(joint_names, tuple):
            raise TypeError('joint_names should be list or tuple, but got {}'.format(type(joint_names)))

        self.joint_names = joint_names
        self.joint_num = len(self.joint_names)

        if not isinstance(actuator_names, list) and not isinstance(actuator_names, tuple):
            raise TypeError('actuator_names should be list or tuple, but got {}'.format(type(actuator_names)))

        self.actuator_names = actuator_names
        self.actuator_num = len(self.actuator_names)

        if self.joint_num != self.actuator_num:
            raise ValueError('joint_num {} and actuator_num {} should be the same'.format(
                self.joint_num, self.actuator_num))

        self._robot_states = None

    def run(self):
        while not rospy.is_shutdown():
            self._get_robot_states()
            self.sim.step()
            if self.viewer is not None:
                self.viewer.render()

    def _get_robot_states(self):
        """Get joint qpos and qvel in each step.

        Returns:
            [qpos, qvel] for each joint given by the name.
        """
        self._robot_states = []

        sim_state = self.sim.get_state()
        for joint_name in self.joint_names:
            joint_qpos_id = self.sim.model.get_joint_qpos_addr(joint_name)
            joint_qpos = sim_state.qpos[joint_qpos_id]
            joint_qvel_id = self.sim.model.get_joint_qvel_addr(joint_name)
            joint_qvel = sim_state.qvel[joint_qvel_id]
            joint_state = [joint_qpos, joint_qvel]
            self._robot_states.append(joint_state)

    def get_joint_states(self):
        """Convert the robot_states to ROS JointState message.

        Returns:

        """
        if not self._robot_states:
            return None
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self._robot_states[:][0]
        joint_state_msg.velocity = self._robot_states[:][1]
        return joint_state_msg

    def set_joint_commands(self, cmd, control_types):
        """

        Args:
            cmd: JointState Joint command message.
            control_types: list[int] How the actuator is controlled.
                           The value could be: position (0), velocity (1), torque (2).
                           It's the user's responsibility to match the type with the actuator's type.
        Returns:

        """
        for i, actuator_name in enumerate(self.actuator_names):
            actuator_id = self.sim.model.actuator_name2id(actuator_name)
            if control_types[i] == 0:
                self.sim.data.ctrl[actuator_id] = cmd.position[i]
            elif control_types[i] == 1:
                self.sim.data.ctrl[actuator_id] = cmd.velocity[i]
            elif control_types[i] == 2:
                self.sim.data.ctrl[actuator_id] = cmd.effort[i]
            else:
                raise TypeError('Unsupported type {}'.format(control_types[i]))
