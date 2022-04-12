from __future__ import print_function

import os
import time
import mujoco

import numpy as np
import xml.etree.ElementTree as ElementTree

from threading import Thread

from rotools.simulation.mujoco.mujoco_viewer import MujocoViewer
from rotools.utility.mjcf import find_elements, find_parent, array_to_string, string_to_array
from rotools.utility.common import to_ros_pose, to_ros_twist, to_list

try:
    import rospy
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Pose
    from rosgraph_msgs.msg import Clock
except ImportError:
    rospy = None
    JointState = None
    Odometry = None
    Pose = None
    Clock = None


class MuJoCoInterface(Thread):
    """This interface using official python binding is the successor of the mujoco-py interface.
    It is recommended to use this one, whereas the former one is archived for reference.
    """

    def __init__(
            self,
            model_path,
            kinematics_path=None,
            actuator_path=None,
            enable_viewer=True,
            **kwargs
    ):
        """Initialize the MuJoCoInterface.

        Args:
            model_path: str Path to the XML file containing the whole model of the robot.
            kinematics_path: str [Optional] Path to the XML file containing the kinematic tree of the robot.
            actuator_path: str [Optional] Path to the XML file containing the actuator and sensor of the robot.
            enable_viewer: bool If true, the MuJoCo Viewer will be displayed.
            **kwargs: DO NOT REMOVE!
        """
        super(MuJoCoInterface, self).__init__()
        Thread.__init__(self)

        self._enable_viewer = enable_viewer

        if not os.path.exists(model_path):
            raise FileNotFoundError("Model XML file '{}' does not exist".format(model_path))
        self._model_path = model_path

        # MuJoCo objects
        self._model = None
        self._data = None
        self._viewer = None

        # These variables all have the same length and correspond with others.
        self._actuated_joint_names = []
        self._actuated_joint_ranges = {}
        self._mimic_joint_names = []
        self.actuator_names = []
        self.control_types = []

        if os.path.exists(kinematics_path):
            kinematic_tree = ElementTree.parse(kinematics_path)
            kinematics_root = kinematic_tree.getroot()

            self.robot_name = find_elements(kinematics_root, 'body').attrib['name']
            if self.robot_name is not None:
                rospy.loginfo("Robot name: {}".format(self.robot_name))
            else:
                raise ValueError("Cannot find body in the kinematic tree")
        else:
            self.robot_name = None
            kinematics_root = None
            rospy.logwarn("Kinematics XML file '{}' does not exist".format(kinematics_path))

        if os.path.exists(actuator_path):
            actuator_tree = ElementTree.parse(actuator_path)
            actuator_root = actuator_tree.getroot()

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
            self._get_actuator_info(position_actuators, 0)
            self._get_actuator_info(velocity_actuators, 1)
            self._get_actuator_info(torque_actuators, 2)

            self._get_mimic_joint_info(mimic_joints)
            self._get_actuated_joint_ranges(kinematics_root)

            self._actuator_num = len(self.actuator_names)
            rospy.loginfo('Controlled joints #{}:\n{}'.format(
                self._actuator_num, array_to_string(self._actuated_joint_names))
            )
            rospy.loginfo(
                'Control types: position (0), velocity (1), torque (2).\n{}'.format(
                    array_to_string(self.control_types)))

            self._null_sensor = 'none'
            self.effort_sensor_names = [self._null_sensor] * self._actuator_num
            self._get_effort_sensor_info(kinematics_root, torque_sensors)
            self._get_effort_sensor_info(kinematics_root, force_sensors)

            rospy.loginfo('Effort sensors:\n{}'.format(array_to_string(self.effort_sensor_names)))

            self._set_initial_state(initial_keyframe)
        else:
            rospy.logwarn("Actuator XML file '{}' does not exist".format(actuator_path))

        self._robot_states = None

        self._clock_publisher = rospy.Publisher('/clock', Clock, queue_size=1)

        self._tracked_object_name = 'object'

    def set_tracked_object(self, name):
        try:
            self._data.body(name)
            self._tracked_object_name = name
        except KeyError:
            rospy.logwarn('Failed to set object to track since the model does not have body {}'.format(name))

    @property
    def n_actuator(self):
        return self._actuator_num

    def run(self):
        # The model, data, and viewer must be initialized here.
        self._model = mujoco.MjModel.from_xml_path(self._model_path)
        self._data = mujoco.MjData(self._model)
        self._viewer = MujocoViewer(self._model, self._data) if self._enable_viewer else None

        start = time.time()
        clock_msg = Clock()
        while not rospy.is_shutdown():
            self._get_robot_states()
            mujoco.mj_step(self._model, self._data)
            if self._viewer is not None:
                self._viewer.render()
            clock_msg.clock = clock_msg.clock.from_sec(time.time() - start)
            self._clock_publisher.publish(clock_msg)
        self._viewer.close()

    def _set_initial_state(self, initial_keyframe):
        if initial_keyframe is not None:
            qpos_str = ' '.join(initial_keyframe.attrib['qpos'].split())
            qpos = string_to_array(qpos_str)
            if len(qpos) != self._model.nq:
                rospy.logwarn(
                    'Keyframe qpos size {} does not match internal state {}'.format(len(qpos), self._model.nq))
                return
            qvel = np.zeros(self._model.nv)
            # TODO
            rospy.loginfo('Set initial state with keyframe values')
        else:
            rospy.loginfo('No initial state from keyframe values')

    def _get_actuator_info(self, actuators, control_type):
        """Given actuator XML elements, get their names and corresponding joint attributes and control types,
        and set member variables: _actuated_joint_names, actuator_names, control_types.

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

    def _get_actuated_joint_ranges(self, kinematics_root):
        if kinematics_root is None:
            return
        for name in self._actuated_joint_names:
            joint = find_elements(kinematics_root, 'joint', {'name': name})
            try:
                joint_range = string_to_array(joint.attrib['range'])
                self._actuated_joint_ranges[name] = joint_range
            except KeyError:
                self._actuated_joint_ranges[name] = None
                # Silently handle actuated joints with no range defined, which could be continuous joints
                pass

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
        if not self._actuated_joint_names:
            return
        robot_states = []
        for i, joint_name in enumerate(self._actuated_joint_names):
            joint = self._data.joint(joint_name)
            joint_qpos = joint.qpos[0]  # qpos and qvel are ndarray with 1 element
            joint_qvel = joint.qvel[0]
            if self.effort_sensor_names[i] != self._null_sensor:
                joint_qtau = self._get_effort_sensor_data(self.effort_sensor_names[i])
            else:
                joint_qtau = 0.
            joint_state = [joint_qpos, joint_qvel, joint_qtau]
            robot_states.append(joint_state)
        self._robot_states = np.array(robot_states)

    def _get_effort_sensor_data(self, sensor_name, axis='z'):
        """Get the force/effort sensor reading for the given axis.

        Args:
            sensor_name: str Name of the force/torque sensor.
            axis: str Name of the axis to get the reading. Could be x, y, z (case insensitive).

        Returns:
            None if the sensor does not return 3 values, otherwise float64.

        Raises:
            NotImplementedError if the axis value is invalid.
        """
        sensor = self._data.sensor(sensor_name)
        if len(sensor.data) != 3:
            rospy.logwarn_throttle(
                1, 'The data number is not 3 for {}, maybe it is not a effort/force sensor?'.format(sensor_name))
            return None
        if axis.lower() == 'x':
            return sensor.data[0]
        elif axis.lower() == 'y':
            return sensor.data[1]
        elif axis.lower() == 'z':
            return sensor.data[2]
        else:
            raise NotImplementedError('Unsupported axis {}, only x, y, z are valid'.format(axis))

    def get_joint_states(self):
        """Convert the robot_states to ROS JointState message.
        The values will be clapped to the joint's ranges if that exist.

        Returns:
            None if the robot_state is not available, otherwise return JointState
        """
        if self._robot_states is None:
            rospy.logwarn_throttle(1, 'MuJoCo robot state has not been set')
            return None
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self._actuated_joint_names
        q_clapped = []
        q_raw = self._robot_states[:, 0].tolist()
        for i, r in enumerate(self._actuated_joint_ranges.values()):
            if r is None:
                q_clapped.append(q_raw[i])
            else:
                if q_raw[i] < r[0]:
                    q_clapped.append(r[0])
                elif q_raw[i] > r[1]:
                    q_clapped.append(r[1])
                else:
                    q_clapped.append(q_raw[i])
        joint_state_msg.position = q_clapped
        joint_state_msg.velocity = self._robot_states[:, 1].tolist()
        joint_state_msg.effort = self._robot_states[:, 2].tolist()
        return joint_state_msg

    def get_odom(self):
        """Get the robot base's pose wrt the world frame.

        Returns:
            Odometry if valid data is present, None otherwise.
        """
        if not self.robot_name or self._data is None:
            # The data could be None when the simulation just started, but this should not last long.
            return None
        odom = Odometry()
        robot_base = self._data.body(self.robot_name)
        xpos = robot_base.xpos
        xquat = robot_base.xquat  # quaternion, w comes first
        cvel = robot_base.cvel  # com-based velocity, 6 entities, 3 for rotation, 3 for translation
        odom.header.stamp = rospy.Time().now()
        pose = to_ros_pose(to_list(xpos) + to_list(xquat), w_first=True)
        twist = to_ros_twist(to_list(cvel[3:]) + to_list(cvel[:3]))
        odom.pose.pose = pose
        odom.twist.twist = twist
        return odom

    def get_object_pose(self):
        """Get the pose of the object's geom.
        The object body's name is hardcoded for now.

        Returns:
            Pose in the world frame.
        """
        if self._data is None:
            # The data could be None when the simulation just started, but this should not last long.
            return None
        try:
            tracked_object = self._data.body(self._tracked_object_name)
            xpos = tracked_object.xpos
            xquat = tracked_object.xquat
            return to_ros_pose(to_list(xpos) + to_list(xquat), w_first=True)
        except KeyError:
            return None

    def set_joint_command(self, cmd):
        """Obtain joint commands according to the internally defined control types and apply the command to the robot.

        Args:
            cmd: JointState Joint command message. It's the user's responsibility to match the command value's type
                 with the actuator's type.
        Returns:
            None
        """
        if not cmd.name:
            for i, actuator_name in enumerate(self.actuator_names):
                actuator = self._data.actuator(actuator_name)
                if self.control_types[i] == 0:
                    actuator.ctrl = cmd.position[i]
                elif self.control_types[i] == 1:
                    actuator.ctrl = cmd.velocity[i]
                elif self.control_types[i] == 2:
                    actuator.ctrl = cmd.effort[i]
                else:
                    raise TypeError(
                        'Unsupported control type {}. Valid types are 0, 1, 2'.format(self.control_types[i]))
            return
        for k, name in enumerate(cmd.name):
            try:
                i = self._actuated_joint_names.index(name)
                actuator = self._data.actuator(self.actuator_names[i])
                if self.control_types[i] == 0:
                    actuator.ctrl = cmd.position[k]
                elif self.control_types[i] == 1:
                    actuator.ctrl = cmd.velocity[k]
                elif self.control_types[i] == 2:
                    actuator.ctrl = cmd.effort[k]
                else:
                    raise TypeError(
                        'Unsupported control type {}. Valid types are 0, 1, 2'.format(self.control_types[i]))
            except ValueError:
                # We allow the name in cmd not present in _actuated_joint_names
                pass

    def set_base_command(self, vel):
        if len(vel) != 4:
            rospy.logwarn_throttle(1, 'Only support 4 velocity commands for wheels')
            return
        wheel_fl = self._data.actuator('WHEEL_FL')
        wheel_fr = self._data.actuator('WHEEL_FR')
        wheel_bl = self._data.actuator('WHEEL_BL')
        wheel_br = self._data.actuator('WHEEL_BR')
        wheel_fl.ctrl, wheel_fr.ctrl, wheel_bl.ctrl, wheel_br.ctrl = vel

    def set_gripper_command(self, device_names, value):
        for device_name in device_names:
            try:
                idx = self._actuated_joint_names.index(device_name)
                actuator = self._data.actuator(self.actuator_names[idx])
                actuator.ctrl = value
            except BaseException as e:
                print(e)
                return False
        return True
