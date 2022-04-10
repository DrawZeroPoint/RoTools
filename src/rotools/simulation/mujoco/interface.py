from __future__ import print_function

import os
import numpy as np
import xml.etree.ElementTree as ElementTree

from threading import Thread
from mujoco_py import MjSim, MjSimState, MjViewer, load_model_from_path

from rotools.utility.mjcf import find_elements, find_parent, array_to_string, string_to_array
from rotools.utility.common import to_ros_pose, to_ros_twist, to_list

try:
    import rospy
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Pose
except ImportError:
    rospy = None
    JointState = None
    Odometry = None
    Pose = None


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

        kinematic_tree = ElementTree.parse(kinematics_path)
        kinematics_root = kinematic_tree.getroot()

        actuator_tree = ElementTree.parse(actuator_path)
        actuator_root = actuator_tree.getroot()

        self.robot_name = find_elements(kinematics_root, 'body').attrib['name']
        if self.robot_name is not None:
            rospy.loginfo("Robot name: {}".format(self.robot_name))
        else:
            raise ValueError("Cannot find body in the kinematic tree")

        position_actuators = find_elements(actuator_root, 'position', return_first=False)
        velocity_actuators = find_elements(actuator_root, 'velocity', return_first=False)
        torque_actuators = find_elements(actuator_root, 'motor', return_first=False)

        mimic_joints = find_elements(actuator_root, 'joint', return_first=False)

        torque_sensors = find_elements(actuator_root, 'torque', return_first=False)
        force_sensors = find_elements(actuator_root, 'force', return_first=False)

        # Set the initial state of the simulation (t=0). Assume the keyframe tag is in the actuator xml
        initial_keyframe = find_elements(actuator_root, 'key')

        """
        control_types: list[int] How the actuator is controlled.
                       The value could be: position (0), velocity (1), torque (2).
        """
        self._actuated_joint_names = []
        self._mimic_joint_names = []
        self.actuator_names = []
        self.control_types = []

        self._get_actuator_info(position_actuators, 0)
        self._get_actuator_info(velocity_actuators, 1)
        self._get_actuator_info(torque_actuators, 2)

        self._get_mimic_joint_info(mimic_joints)

        self._actuator_num = len(self.actuator_names)
        self._actuator_ids = [self.sim.model.actuator_name2id(actuator_name) for actuator_name in self.actuator_names]

        self._wheel_actuator_ids = self._get_wheel_actuator_info()

        rospy.loginfo('Controlled joints #{}:\n{}'.format(
            self._actuator_num, array_to_string(self._actuated_joint_names))
        )
        rospy.loginfo('Control types:\n{}'.format(array_to_string(self.control_types)))

        self._null_sensor = 'none'
        self.effort_sensor_names = [self._null_sensor] * self._actuator_num
        self._get_effort_sensor_info(kinematics_root, torque_sensors)
        self._get_effort_sensor_info(kinematics_root, force_sensors)

        rospy.loginfo('Effort sensors:\n{}'.format(array_to_string(self.effort_sensor_names)))

        self._set_initial_state(initial_keyframe)

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

    def _set_initial_state(self, initial_keyframe):
        if initial_keyframe is not None:
            qpos_str = ' '.join(initial_keyframe.attrib['qpos'].split())
            qpos = string_to_array(qpos_str)
            if len(qpos) != self.model.nq:
                rospy.logwarn('Keyframe qpos size {} does not match internal state {}'.format(len(qpos), self.model.nq))
                return
            qvel = np.zeros(self.model.nv)
            old_state = self.sim.get_state()
            new_state = MjSimState(0, qpos, qvel, old_state.act, old_state.udd_state)
            self.sim.set_state(new_state)
            self.sim.forward()
            rospy.loginfo('Set initial state with keyframe values')
        else:
            rospy.loginfo('No initial state from keyframe values')

    def _get_actuator_info(self, actuators, control_type):
        """Given actuator XML elements, get their names and corresponding joint attributes and control types,
        and set member variables.

        Args:
            actuators: list of ET.Element Actuator elements.
            control_type: int Control type.

        Returns:
            None
        """
        if actuators is not None:
            for actuator in actuators:
                self._actuated_joint_names.append(actuator.attrib['joint'])
                self.actuator_names.append(actuator.attrib['name'])
                self.control_types.append(control_type)

    def _get_mimic_joint_info(self, mimic_joint_elements):
        for name in self._actuated_joint_names:
            matched = False
            for element in mimic_joint_elements:
                joint1 = element.attrib['joint1']
                joint2 = element.attrib['joint2']
                if joint1 == name:
                    self._mimic_joint_names.append(joint2)
                    matched = True
                    break
            if not matched:
                self._mimic_joint_names.append(None)

    def _get_wheel_actuator_info(self):
        wheel_actuator_ids = {'WHEEL_FR': -1, 'WHEEL_FL': -1, 'WHEEL_BL': -1, 'WHEEL_BR': -1}
        for i, name in enumerate(self.actuator_names):
            if name in wheel_actuator_ids.keys():
                wheel_actuator_ids[name] = self._actuator_ids[i]
        assert -1 not in wheel_actuator_ids.items(), rospy.logwarn(1, 'Not all wheel actuator found')
        return wheel_actuator_ids

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
            [qpos, qvel, qtau] for each joint given by the name.
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
            rospy.logwarn('MuJoCo robot state has not been set')
            return None
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self._actuated_joint_names
        joint_state_msg.position = self._robot_states[:, 0].tolist()
        joint_state_msg.velocity = self._robot_states[:, 1].tolist()
        joint_state_msg.effort = self._robot_states[:, 2].tolist()
        return joint_state_msg

    def get_odom(self):
        """Get the robot base's pose wrt the world frame.

        Returns:
            Odometry
        """
        odom = Odometry()
        xpos = self.sim.data.get_body_xpos(self.robot_name)
        xquat = self.sim.data.get_body_xquat(self.robot_name)  # w comes first
        xvelp = self.sim.data.get_body_xvelp(self.robot_name)
        xvelr = self.sim.data.get_body_xvelr(self.robot_name)
        odom.header.stamp = rospy.Time().now()
        pose = to_ros_pose(to_list(xpos) + to_list(xquat), w_first=True)
        twist = to_ros_twist(to_list(xvelp) + to_list(xvelr))
        odom.pose.pose = pose
        odom.twist.twist = twist
        return odom

    def get_object_pose(self):
        """Get the pose of the object's geom.
        The object body's name is hardcoded for now.

        Returns:
            Pose in the world frame.
        """
        xpos = self.sim.data.get_body_xpos('object')
        xquat = self.sim.data.get_body_xquat('object')
        return to_ros_pose(to_list(xpos) + to_list(xquat), w_first=True)

    def set_joint_command(self, cmd):
        """Obtain joint commands according to the internally defined control types and apply the command to the robot.

        Args:
            cmd: JointState Joint command message. It's the user's responsibility to match the command value's type
                 with the actuator's type.
        Returns:
            None
        """
        if not cmd.name:
            for i, actuator_id in enumerate(self._actuator_ids):
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
                actuator_id = self._actuator_ids[i]
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

    def set_base_command(self, vel):
        if len(vel) != 4:
            rospy.logwarn('Only support 4 velocity commands for wheels')
            return

        self.sim.data.ctrl[[
            self._wheel_actuator_ids['WHEEL_FL'], self._wheel_actuator_ids['WHEEL_FR'],
            self._wheel_actuator_ids['WHEEL_BL'], self._wheel_actuator_ids['WHEEL_BR']
        ]] = vel

    def set_gripper_command(self, device_names, value):
        for device_name in device_names:
            try:
                idx = self._actuated_joint_names.index(device_name)
                self.sim.data.ctrl[self._actuator_ids[idx]] = value
            except ValueError:
                return False
        return True
