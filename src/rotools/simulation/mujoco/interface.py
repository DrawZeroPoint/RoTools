from __future__ import print_function

import os
import numpy as np
import xml.etree.ElementTree as ElementTree

from threading import Thread
from mujoco_py import MjSim, MjViewer, load_model_from_path

from rotools.utility.mjcf import find_elements, find_parent, array_to_string

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
            kinematics_path,
            actuator_path,
            enable_viewer=True,
            **kwargs
    ):
        """Initialize the MuJoCoInterface.

        Args:
            model_path: str Path to the XML file containing the whole model of the robot.
            kinematics_path: str Path to the XML file containing the kinematic tree of the robot.
            actuator_path: str Path to the XML file containing the actuator and sensor definitions of the robot.
            enable_viewer: bool If true, the MuJoCo Viewer will be displayed.
            **kwargs: DO NOT REMOVE!
        """
        super(MuJoCoInterface, self).__init__()
        Thread.__init__(self)

        if not os.path.exists(model_path):
            raise FileNotFoundError("Model XML file '{}' does not exist".format(model_path))

        self.model = load_model_from_path(model_path)
        self.sim = MjSim(self.model)

        if not os.path.exists(kinematics_path):
            raise FileNotFoundError("Kinematics XML file '{}' does not exist".format(kinematics_path))

        if not os.path.exists(actuator_path):
            raise FileNotFoundError("Actuator XML file '{}' does not exist".format(actuator_path))

        kinematics_tree = ElementTree.parse(kinematics_path)
        kinematics_root = kinematics_tree.getroot()

        actuator_tree = ElementTree.parse(actuator_path)
        actuator_root = actuator_tree.getroot()

        position_actuators = find_elements(actuator_root, 'position', return_first=False)
        velocity_actuators = find_elements(actuator_root, 'velocity', return_first=False)
        torque_actuators = find_elements(actuator_root, 'motor', return_first=False)

        torque_sensors = find_elements(actuator_root, 'torque', return_first=False)
        force_sensors = find_elements(actuator_root, 'force', return_first=False)

        """
        control_types: list[int] How the actuator is controlled.
                       The value could be: position (0), velocity (1), torque (2).
        """
        self._actuated_joint_names = []
        self.actuator_names = []
        self.control_types = []
        self._get_actuator_info(position_actuators, 0)
        self._get_actuator_info(velocity_actuators, 1)
        self._get_actuator_info(torque_actuators, 2)

        self._actuator_num = len(self.actuator_names)
        self.actuator_ids = [self.sim.model.actuator_name2id(actuator_name) for actuator_name in self.actuator_names]

        rospy.loginfo('Controlled joints #{}:\n{}'.format(
            self._actuator_num, array_to_string(self._actuated_joint_names))
        )
        rospy.loginfo('Control types:\n{}'.format(array_to_string(self.control_types)))

        self._null_sensor = 'none'
        self.effort_sensor_names = [self._null_sensor] * self._actuator_num
        self._get_effort_sensor_info(kinematics_root, torque_sensors)
        self._get_effort_sensor_info(kinematics_root, force_sensors)

        rospy.loginfo('Effort sensors:\n{}'.format(array_to_string(self.effort_sensor_names)))

        self.viewer = MjViewer(self.sim) if enable_viewer else None

        self._robot_states = None

    @property
    def n_actuator(self):
        return self._actuator_num

    def run(self):
        while not rospy.is_shutdown():
            self._get_robot_states()
            self.sim.step()
            if self.viewer is not None:
                self.viewer.render()

    def _get_actuator_info(self, actuators, control_type):
        if actuators is not None:
            for actuator in actuators:
                self._actuated_joint_names.append(actuator.attrib['joint'])
                self.actuator_names.append(actuator.attrib['name'])
                self.control_types.append(control_type)

    def _get_effort_sensor_info(self, kinematics_root, sensors):
        if sensors is not None:
            for sensor in sensors:
                site_name = sensor.attrib['site']
                site = find_elements(kinematics_root, 'site', {'name': site_name})
                if site is None:
                    rospy.logwarn('Site {} is not in the kinematic tree'.format(site_name))
                    continue
                parent = find_parent(kinematics_root, site)
                if parent is not None:
                    joint = find_elements(parent, 'joint')
                    if joint is not None:
                        joint_name = joint.attrib['name']
                        try:
                            index = self._actuated_joint_names.index(joint_name)
                            self.effort_sensor_names[index] = sensor.attrib['name']
                        except ValueError:
                            rospy.logwarn('Sensor {} corresponds to non-actuated joint {}'.format
                                          (sensor.attrib['name'], joint_name))
                    else:
                        rospy.logwarn('Sensor {} have no corresponding joint'.format(sensor.attrib['name']))
                else:
                    rospy.logwarn('Cannot find parent for site {}'.format(site_name))

    def _get_robot_states(self):
        """Get robot joint states for each step.

        Returns:
            [qpos, qvel, q_tau_j] for each joint given by the name.
        """
        robot_states = []
        sim_state = self.sim.get_state()
        for i, joint_name in enumerate(self._actuated_joint_names):
            joint_qpos_id = self.sim.model.get_joint_qpos_addr(joint_name)
            joint_qpos = sim_state.qpos[joint_qpos_id]
            joint_qvel_id = self.sim.model.get_joint_qvel_addr(joint_name)
            joint_qvel = sim_state.qvel[joint_qvel_id]
            if self.effort_sensor_names[i] != self._null_sensor:
                joint_effort = self._get_effort_sensor_data(self.effort_sensor_names[i])
            else:
                joint_effort = 0.
            joint_state = [joint_qpos, joint_qvel, joint_effort]
            robot_states.append(joint_state)
        self._robot_states = np.array(robot_states)

    def _get_effort_sensor_data(self, sensor_name):
        """This is a fix for the self.sim.data.get_sensor method in mujoco-py.

        Args:
            sensor_name: str Name of the force/torque sensor.

        Returns:
            None if IndexError, otherwise float64.
        """
        sensor_id = self.sim.model.sensor_name2id(sensor_name)
        try:
            effort_data = self.sim.data.sensordata[sensor_id * 3: sensor_id * 3 + 3]
            return effort_data[-1]  # the force/torque along/around the z-axis is what we care
        except IndexError:
            rospy.logerr('Due to a known bug of mujoco-py (see: https://github.com/openai/mujoco-py/issues/684), '
                         'we currently only support sensors with 3 outputs (torque/force)')
            return None

    def get_joint_states(self):
        """Convert the robot_states to ROS JointState message.

        Returns:
            None if the robot_state is not available, otherwise return JointState
        """
        if self._robot_states is None:
            return None
        joint_state_msg = JointState()
        joint_state_msg.name = self._actuated_joint_names
        joint_state_msg.position = self._robot_states[:, 0].tolist()
        joint_state_msg.velocity = self._robot_states[:, 1].tolist()
        joint_state_msg.effort = self._robot_states[:, 2].tolist()
        return joint_state_msg

    def set_joint_commands(self, cmd):
        """Obtain joint commands according to the internally defined control types and apply the command to the robot.

        Args:
            cmd: JointState Joint command message. It's the user's responsibility to match the command value's type
                 with the actuator's type.
        Returns:
            None
        """
        if not cmd.name:
            for i, actuator_id in enumerate(self.actuator_ids):
                if self.control_types[i] == 0:
                    self.sim.data.ctrl[actuator_id] = cmd.position[i]
                elif self.control_types[i] == 1:
                    self.sim.data.ctrl[actuator_id] = cmd.velocity[i]
                elif self.control_types[i] == 2:
                    self.sim.data.ctrl[actuator_id] = cmd.effort[i]
                else:
                    raise TypeError('Unsupported type {}'.format(self.control_types[i]))
            return
        for k, name in enumerate(cmd.name):
            try:
                i = self._actuated_joint_names.index(name)
                actuator_id = self.actuator_ids[i]
                if self.control_types[i] == 0:
                    self.sim.data.ctrl[actuator_id] = cmd.position[k]
                elif self.control_types[i] == 1:
                    self.sim.data.ctrl[actuator_id] = cmd.velocity[k]
                elif self.control_types[i] == 2:
                    self.sim.data.ctrl[actuator_id] = cmd.effort[k]
                else:
                    raise TypeError('Unsupported type {}'.format(self.control_types[i]))
            except ValueError:
                # We allow the name in cmd not present in _actuated_joint_names
                pass
