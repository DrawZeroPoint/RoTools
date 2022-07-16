from __future__ import print_function

import os
import time
import mujoco

import numpy as np
import xml.etree.ElementTree as ElementTree

from threading import Thread

from rotools.simulation.mujoco.mujoco_viewer import MujocoViewer
from rotools.utility.mjcf import (
    find_elements,
    find_parent,
    array_to_string,
    string_to_array,
)
from rotools.utility.common import (
    to_ros_pose,
    to_ros_twist,
    to_list,
    all_close,
    get_transform_same_origin,
    to_ros_orientation,
)
from rotools.utility.color_palette import bwr_color_palette

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
        verbose=False,
        **kwargs
    ):
        """Initialize the MuJoCoInterface.

        Args:
            model_path: str Path to the XML file containing the whole model of the robot.
            kinematics_path: str [Optional] Path to the XML file containing the kinematic tree of the robot.
            actuator_path: str [Optional] Path to the XML file containing the actuator and sensor of the robot.
            enable_viewer: bool If true, the MuJoCo Viewer will be displayed.
            verbose: bool If true, detailed information will be shown.
            **kwargs: DO NOT REMOVE!
        """
        super(MuJoCoInterface, self).__init__()
        Thread.__init__(self)

        self._enable_viewer = enable_viewer

        if not os.path.exists(model_path):
            raise FileNotFoundError(
                "Model XML file '{}' does not exist".format(model_path)
            )
        self._model_path = model_path

        # MuJoCo objects
        self._model = None
        self._data = None
        self._viewer = None

        # The following list/dicts all have the same length and correspond with others.
        self._actuated_joint_names = []
        self._actuated_joint_ranges = {}
        self._actuated_joint_geoms = {}
        self._mimic_joint_names = []
        self.actuator_names = []

        # We consider the ctrl range of the actuator could be different with that of the joint.
        self._actuator_ctrl_ranges = {}
        self._actuator_force_ranges = {}
        self.control_types = []

        if os.path.exists(kinematics_path):
            kinematic_tree = ElementTree.parse(kinematics_path)
            kinematics_root = kinematic_tree.getroot()
            # This robot name will also refer to the base frame of the robot
            self.robot_name = find_elements(kinematics_root, "body").attrib["name"]
            if self.robot_name is not None:
                rospy.loginfo("Robot name: {}".format(self.robot_name))
            else:
                raise ValueError("Cannot find body in the kinematic tree")
        else:
            self.robot_name = None
            kinematics_root = None
            rospy.logwarn(
                "Kinematics XML file '{}' does not exist".format(kinematics_path)
            )

        if os.path.exists(actuator_path):
            actuator_tree = ElementTree.parse(actuator_path)
            actuator_root = actuator_tree.getroot()

            position_actuators = find_elements(
                actuator_root, "position", return_first=False
            )
            velocity_actuators = find_elements(
                actuator_root, "velocity", return_first=False
            )
            torque_actuators = find_elements(actuator_root, "motor", return_first=False)

            mimic_joints = find_elements(actuator_root, "joint", return_first=False)

            torque_sensors = find_elements(actuator_root, "torque", return_first=False)
            force_sensors = find_elements(actuator_root, "force", return_first=False)

            """
            control_types: list[int] How the actuator is controlled.
                           The value could be: position (0), velocity (1), torque (2).
            """
            self._get_actuator_info(position_actuators, 0)
            self._get_actuator_info(velocity_actuators, 1)
            self._get_actuator_info(torque_actuators, 2)

            self._get_mimic_joint_info(mimic_joints)
            self._get_actuated_joint_ranges(kinematics_root)
            self._get_actuator_ranges(actuator_root)
            self._get_actuated_joint_geoms(kinematics_root)

            self._actuator_num = len(self.actuator_names)
            rospy.loginfo(
                "Controlled joints #{}:\n{}".format(
                    self._actuator_num, array_to_string(self._actuated_joint_names)
                )
            )
            rospy.loginfo(
                "Control types: position (0), velocity (1), torque (2).\n{}".format(
                    array_to_string(self.control_types)
                )
            )

            self._null_sensor = "none"
            self.effort_sensor_names = [self._null_sensor] * self._actuator_num
            self._get_effort_sensor_info(kinematics_root, torque_sensors)
            self._get_effort_sensor_info(kinematics_root, force_sensors)

            rospy.loginfo(
                "Effort sensors:\n{}".format(array_to_string(self.effort_sensor_names))
            )

        else:
            rospy.logwarn("Actuator XML file '{}' does not exist".format(actuator_path))

        self._robot_states = None

        self._clock_publisher = rospy.Publisher("/clock", Clock, queue_size=1)

        self._tracked_object_name = "object"

        self._neutral_initialized = False

        self.verbose = verbose
        self.reset_verbose = False

    def set_tracked_object(self, name):
        try:
            self._data.body(name)
            self._tracked_object_name = name
        except KeyError:
            rospy.logwarn(
                "Failed to set object to track since the model does not have body {}".format(
                    name
                )
            )

    @property
    def n_actuator(self):
        return self._actuator_num

    def run(self):
        # The model, data, and viewer must be initialized here.
        self._model = mujoco.MjModel.from_xml_path(self._model_path)
        # Optionally disable the 'anchor' connection of the object at the beginning
        try:
            self._model.equality("anchor").active = False
        except KeyError:
            pass
        self._data = mujoco.MjData(self._model)
        self._viewer = (
            MujocoViewer(self._model, self._data) if self._enable_viewer else None
        )

        start = time.time()
        clock_msg = Clock()
        while not rospy.is_shutdown():
            if not self._neutral_initialized:
                self._neutral_initialization()
                self._neutral_initialized = True
            self._get_robot_states()
            mujoco.mj_step(self._model, self._data)
            if self._viewer is not None:
                self._viewer.render()
            clock_msg.clock = clock_msg.clock.from_sec(time.time() - start)
            self._clock_publisher.publish(clock_msg)
        self._viewer.close()

    def _neutral_initialization(self):
        """Set the initial control signal for the actuator as 'neutral', which means that it must lie in the
        limit range of the control. This prevents the initial configuration of the robot violate the limits.
        The initialization only applies to the position controlled actuators with ctrl range defined.

        Returns:
            None
        """
        for i, t in enumerate(self.control_types):
            if t == 0:
                actuator_name = self.actuator_names[i]
                ctrl_range = self._actuator_ctrl_ranges[actuator_name]
                if ctrl_range is not None:
                    low, high = ctrl_range
                    if 0.0 <= low:
                        neutral = low + (high - low) / 10.0
                    elif high <= 0.0:
                        neutral = high - (high - low) / 10.0
                    else:
                        neutral = 0.0
                    actuator = self._data.actuator(actuator_name)
                    actuator.ctrl = neutral

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
                self._actuated_joint_names.append(actuator.attrib["joint"])
                self.actuator_names.append(actuator.attrib["name"])
                self.control_types.append(control_type)

    def _get_mimic_joint_info(self, mimic_joint_elements):
        for name in self._actuated_joint_names:
            matched = False
            for element in mimic_joint_elements:
                joint1 = element.attrib["joint1"]
                joint2 = element.attrib["joint2"]
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
            joint = find_elements(kinematics_root, "joint", {"name": name})
            try:
                joint_range = string_to_array(joint.attrib["range"])
                self._actuated_joint_ranges[name] = joint_range
            except KeyError:
                # Silently handle actuated joints with no range defined, which could be continuous joints
                self._actuated_joint_ranges[name] = None

    def _get_actuated_joint_geoms(self, kinematics_root):
        if kinematics_root is None:
            return
        for name in self._actuated_joint_names:
            joint = find_elements(kinematics_root, "joint", {"name": name})
            try:
                parent_body = find_parent(kinematics_root, joint)
                geom = find_elements(parent_body, "geom")
                geom_name = geom.attrib["name"]
                self._actuated_joint_geoms[name] = geom_name
            except KeyError:
                self._actuated_joint_geoms[name] = None

    def _get_actuator_ranges(self, actuator_root):
        """Get the actuators' control ranges & force ranges and store them to the class members
        _actuator_ctrl_ranges and _actuator_force_ranges. The control is the input of the actuator,
        and the force is the output of the actuator (no matter which one it is).

        Args:
            actuator_root: ET.Element The xml element tree to start recursively searching actuators.

        Returns:
            None

        Raises:
            NotImplementedError if the control type is not in (0, 1, 2).
        """
        if actuator_root is None:
            return
        for i, name in enumerate(self.actuator_names):
            if self.control_types[i] == 0:
                actuator = find_elements(actuator_root, "position", {"name": name})
            elif self.control_types[i] == 1:
                actuator = find_elements(actuator_root, "velocity", {"name": name})
            elif self.control_types[i] == 2:
                actuator = find_elements(actuator_root, "motor", {"name": name})
            else:
                raise NotImplementedError

            try:
                ctrl_range = string_to_array(actuator.attrib["ctrlrange"])
                self._actuator_ctrl_ranges[name] = ctrl_range
            except KeyError:
                # Silently handle actuators with no ctrl range defined
                self._actuator_ctrl_ranges[name] = None

            try:
                force_range = string_to_array(actuator.attrib["forcerange"])
                self._actuator_force_ranges[name] = force_range
            except KeyError:
                # Silently handle actuators with no ctrl range defined
                self._actuator_force_ranges[name] = None

    def _get_effort_sensor_info(self, kinematics_root, sensors):
        if sensors is not None:
            for sensor in sensors:
                site_name = sensor.attrib["site"]
                site = find_elements(kinematics_root, "site", {"name": site_name})
                if site is None:
                    rospy.logwarn(
                        "Site {} is not in the kinematic tree".format(site_name)
                    )
                    continue
                parent = find_parent(kinematics_root, site)
                if parent is not None:
                    joint = find_elements(parent, "joint")
                    if joint is not None:
                        joint_name = joint.attrib["name"]
                        try:
                            index = self._actuated_joint_names.index(joint_name)
                            self.effort_sensor_names[index] = sensor.attrib["name"]
                        except ValueError:
                            rospy.logwarn(
                                "Sensor {} corresponds to non-actuated joint {}".format(
                                    sensor.attrib["name"], joint_name
                                )
                            )
                    else:
                        rospy.logwarn(
                            "Sensor {} have no corresponding joint".format(
                                sensor.attrib["name"]
                            )
                        )
                else:
                    rospy.logwarn("Cannot find parent for site {}".format(site_name))

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
                joint_qtau = 0.0
            joint_state = [joint_qpos, joint_qvel, joint_qtau]
            robot_states.append(joint_state)

            if not self.verbose:
                continue
            force_range = self._actuator_force_ranges[self.actuator_names[i]]
            try:
                geom_name = self._actuated_joint_geoms[joint_name]
            except KeyError:
                geom_name = None

            if force_range is not None:
                low, high = force_range
                level = 0.5
                need_update = False
                if joint_qtau < low:
                    rospy.logwarn(
                        "{} effort {:.4f} is lower than limit {:.4f}".format(
                            joint_name, joint_qtau, low
                        )
                    )
                    level = max(min((low - joint_qtau) / (high - low) * 2, -0.5), -1)
                    need_update = True
                if joint_qtau > high:
                    rospy.logwarn(
                        "{} effort {:.4f} is larger than limit {:.4f}".format(
                            joint_name, joint_qtau, high
                        )
                    )
                    level = min(max((joint_qtau - high) / (high - low) * 2, 0.5), 1)
                    need_update = True
                if self.reset_verbose:
                    need_update = True
                if geom_name is not None and need_update:
                    geom = self._model.geom(geom_name)
                    geom.rgba = bwr_color_palette(level)

        self.reset_verbose = False
        self._robot_states = np.array(robot_states)

    def _get_effort_sensor_data(self, sensor_name, axis="z"):
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
                1,
                "The data number is not 3 for {}, maybe it is not a effort/force sensor?".format(
                    sensor_name
                ),
            )
            return None
        if axis.lower() == "x":
            return sensor.data[0]
        elif axis.lower() == "y":
            return sensor.data[1]
        elif axis.lower() == "z":
            return sensor.data[2]
        else:
            raise NotImplementedError(
                "Unsupported axis {}, only x, y, z are valid".format(axis)
            )

    def get_joint_states(self):
        """Convert the robot_states to ROS JointState message.
        The values will be clapped to the joint's ranges if that exist.

        Returns:
            None if the robot_state is not available, otherwise return JointState
        """
        if self._robot_states is None:
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
        cvel = (
            robot_base.cvel
        )  # com-based velocity, 6 entities, 3 for rotation, 3 for translation
        odom.header.stamp = rospy.Time().now()
        pose = to_ros_pose(to_list(xpos) + to_list(xquat), w_first=True)
        twist = to_ros_twist(to_list(cvel[3:]) + to_list(cvel[:3]))
        odom.pose.pose = pose
        odom.twist.twist = twist
        return odom

    def get_object_pose(self):
        """Get the pose of the object's geom. This object is to be manipulated by the robot.
        The object body's name (object) is hardcoded for now.

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

    def get_site_pose(self, site_name, ref_name=""):
        """Get the relative pose of a site with regard to the reference frame.

        Args:
            site_name: str Name of the site.
            ref_name: str Name of the reference body. If it is empty or None,
                      the base body of the robot will be used.

        Returns:
            Pose/None
        """
        if self._data is None:
            return None
        site_pose = self._get_site_pose(site_name)
        if ref_name == "" or ref_name is None:
            ref_pose = self._get_body_pose(self.robot_name)
        else:
            ref_pose = self._get_body_pose(ref_name)
        if site_pose is not None and ref_pose is not None:
            return to_ros_pose(get_transform_same_origin(ref_pose, site_pose))
        else:
            return None

    def _get_site_pose(self, site_name):
        if self._data is None:
            return None
        try:
            target_site = self._data.site(site_name)
            print(target_site.xpos, target_site.xmat)
            xpos = target_site.xpos
            xquat = to_ros_orientation(target_site.xmat)
            return to_ros_pose(to_list(xpos) + to_list(xquat))
        except KeyError:
            rospy.logerr("Site {} is not exist".format(site_name))
            return None

    def _get_body_pose(self, body_name):
        if self._data is None:
            return None
        try:
            target_body = self._data.body(body_name)
            xpos = target_body.xpos
            xquat = target_body.xquat
            return to_ros_pose(to_list(xpos) + to_list(xquat), w_first=True)
        except KeyError:
            rospy.logerr("Body {} is not exist".format(body_name))
            return None

    def set_joint_command(self, cmd):
        """Obtain joint commands according to the internally defined control types and apply the command to the robot.

        Args:
            cmd: JointState Joint command message. It's the user's responsibility to match the command value's type
                 with the actuator's type.
        Returns:
            None

        Raises:
            TypeError if the class member control_type is not valid.
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
                        "Unsupported control type {}. Valid types are 0, 1, 2".format(
                            self.control_types[i]
                        )
                    )
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
                        "Unsupported control type {}. Valid types are 0, 1, 2".format(
                            self.control_types[i]
                        )
                    )
            except ValueError:
                # We allow the name in cmd not present in _actuated_joint_names
                pass

    def set_base_command(self, vel, factor=1.4):
        """Set velocity commands to wheel of the base.

        Args:
            vel: list[double] A 4-element list containing velocity commands for the 4 wheels:
                 FrontLeft, FrontRight, BackLeft, BackRight.
            factor: double A value to be multiplied to the vel to migrate the sim-to-real gap.

        Returns:
            bool True if succeeded, false otherwise.
        """
        if len(vel) != 4:
            rospy.logwarn_throttle(1, "Only support 4 velocity commands for wheels")
            return False
        wheel_fl = self._data.actuator("WHEEL_FL")
        wheel_fr = self._data.actuator("WHEEL_FR")
        wheel_bl = self._data.actuator("WHEEL_BL")
        wheel_br = self._data.actuator("WHEEL_BR")
        wheel_fl.ctrl, wheel_fr.ctrl, wheel_bl.ctrl, wheel_br.ctrl = vel * factor
        return True

    def set_gripper_command(self, joint_names, cmd_values):
        """Set commands for the gripper containing joints denoted by joint_names.

        Args:
            joint_names: list[str] Names of the joints belonging to one or more grippers.
            cmd_values: float/list[float] If a float is given, all joints will be commanded with this value;
                        If a list is given, each of them will be set to a corresponding joint.

        Returns:
            bool True if all joints meets the commands, False otherwise.
        """
        if isinstance(cmd_values, float):
            cmd_values = [cmd_values] * len(joint_names)
        elif isinstance(cmd_values, list) or isinstance(cmd_values, tuple):
            assert len(joint_names) == len(cmd_values)
        else:
            raise NotImplementedError(
                "cmd_values type {} is not supported".format(type(cmd_values))
            )

        for joint_name, cmd_value in zip(joint_names, cmd_values):
            try:
                idx = self._actuated_joint_names.index(joint_name)
                actuator = self._data.actuator(self.actuator_names[idx])
                actuator.ctrl = cmd_value
            except BaseException as e:
                rospy.logwarn(e)
                return False

        timeout = time.time() + 5.0  # Wait for 5 secs
        while True:
            qpos_list = [
                self._data.joint(joint_name).qpos.tolist()[0]
                for joint_name in joint_names
            ]
            if all_close(qpos_list, cmd_values, 2.0e-3):
                break
            if time.time() > timeout:
                break
            rospy.loginfo_throttle(
                2,
                "Gripper is reaching to {} (current {})".format(cmd_values, qpos_list),
            )
            rospy.sleep(0.1)
        return True

    def reset_object_pose(self):
        """Reset an object's pose to be aligned with that of a fixed body by activating the weld equality
        connection between the two bodies. The name 'anchor' is hardcoded for now

        Returns:
            True if succeeded, false otherwise.
        """
        if self._data is None:
            # The data could be None when the simulation just started, but this should not last long.
            return False
        try:
            self._model.equality("anchor").active = True
            rospy.sleep(1.0)
            self._model.equality("anchor").active = False
            return True
        except KeyError:
            return False
