#!/usr/bin/env python
from __future__ import print_function

import rospy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

from roport.srv import *

import rotools.simulation.mujoco.interface as interface
from rotools.utility.robotics import mecanum_base_get_wheel_velocities


class MuJoCoServer(object):
    """This RoPort server using the RoTools MuJoCoInterface to provide some
    handy services for interacting with the simulated robot in the MuJoCo simulator.
    """

    def __init__(self, kwargs):
        """Initialize the MuJoCoServer.

        Args:
            kwargs: dict Configurations.
        """
        super(MuJoCoServer, self).__init__()

        self.interface = interface.MuJoCoInterface(**kwargs)
        self.interface.start()

        # Provided services
        self._execute_gripper_control_srv = rospy.Service("execute_gripper_control", ExecuteBinaryAction,
                                                          self.execute_gripper_control_handle)

        self._execute_object_reset_srv = rospy.Service("execute_object_reset", ExecuteBinaryAction,
                                                       self.execute_object_reset_handle)

        self._execute_verbose_reset_srv = rospy.Service("execute_verbose_reset", ExecuteBinaryAction,
                                                        self.execute_verbose_reset_handle)

        self._get_pose_srv = rospy.Service("get_group_pose", GetGroupPose, self.get_pose_handle)

        # Received msgs
        if 'joint_command_topic_id' in kwargs.keys():
            joint_command_topic_id = kwargs['joint_command_topic_id']
            self.joint_command_subscriber = rospy.Subscriber(joint_command_topic_id, JointState, self.joint_command_cb)

        if 'base_command_topic_id' in kwargs.keys():
            base_command_topic_id = kwargs['base_command_topic_id']
            self.base_command_subscriber = rospy.Subscriber(base_command_topic_id, Twist, self.base_command_cb)

        # Published msgs
        rate = kwargs['publish_rate'] if 'publish_rate' in kwargs.keys() else 60

        if 'joint_state_topic_id' in kwargs.keys():
            joint_state_topic_id = kwargs['joint_state_topic_id']
            self.joint_state_publisher = rospy.Publisher(joint_state_topic_id, JointState, queue_size=1)

        if 'odom_topic_id' in kwargs.keys():
            odom_topic_id = kwargs['odom_topic_id']
            self.odom_publisher = rospy.Publisher(odom_topic_id, Odometry, queue_size=1)
        else:
            self.odom_publisher = None

        if 'object_pose_topic_id' in kwargs.keys():
            object_pose_topic_id = kwargs['object_pose_topic_id']
            self.object_pose_publisher = rospy.Publisher(object_pose_topic_id, Pose, queue_size=1)
        else:
            self.object_pose_publisher = None

        self._timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.publish_handle)
        if kwargs['verbose']:
            self._verbose_reset_timer = rospy.Timer(rospy.Duration.from_sec(5), self.execute_verbose_reset_handle)

        # Get robot base params if available
        self.wheel_radius = kwargs['wheel_radius'] if 'wheel_radius' in kwargs.keys() else None
        self.base_width = kwargs['base_width'] if 'base_width' in kwargs.keys() else None
        self.base_length = kwargs['base_length'] if 'base_length' in kwargs.keys() else None

    def publish_handle(self, _):
        joint_state_msg = self.interface.get_joint_states()
        if joint_state_msg:
            self.joint_state_publisher.publish(joint_state_msg)

        if self.odom_publisher is not None:
            odom_msg = self.interface.get_odom()
            if odom_msg:
                self.odom_publisher.publish(odom_msg)

        if self.object_pose_publisher is not None:
            object_pose_msg = self.interface.get_object_pose()
            if object_pose_msg:
                self.object_pose_publisher.publish(object_pose_msg)

    def joint_command_cb(self, cmd):
        if len(cmd.position) != len(cmd.velocity) or len(cmd.position) != len(cmd.effort):
            rospy.logwarn_throttle(3, 'Joint command should contain position, velocity, and effort. '
                                      'Their dimensions should be the same')
            return
        if not cmd.name:
            rospy.logwarn_throttle(3, 'Sending joint command with no name is highly discouraged')
            if len(cmd.position) != self.interface.n_actuator:
                rospy.logerr('Joint command size {} and actuator number {} mismatch, omitted.'.format(
                    len(cmd.position), self.interface.n_actuator))
                return

        self.interface.set_joint_command(cmd)

    def base_command_cb(self, cmd):
        if self.wheel_radius is None or self.base_width is None or self.base_length is None:
            rospy.logwarn_throttle(1, 'Base param not set: wheel_radius {}, base_width {}, base_length {}'.format(
                self.wheel_radius, self.base_width, self.base_length))
            return
        vel = mecanum_base_get_wheel_velocities(cmd, self.wheel_radius, self.base_width, self.base_length)
        self.interface.set_base_command(vel)

    def execute_gripper_control_handle(self, req):
        resp = ExecuteBinaryActionResponse()
        # req = ExecuteBinaryActionRequest()
        ok = self.interface.set_gripper_command(req.device_names, req.value)
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def execute_object_reset_handle(self, req):
        resp = ExecuteBinaryActionResponse()
        # req = ExecuteBinaryActionRequest()
        ok = self.interface.reset_object_pose()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def execute_verbose_reset_handle(self, req):
        resp = ExecuteBinaryActionResponse()
        self.interface.reset_verbose = True
        resp.result_status = resp.SUCCEEDED
        return resp

    def get_pose_handle(self, req):
        resp = GetGroupPoseResponse()
        ee_pose = self.interface.get_site_pose(req.ee_frame, req.ref_frame)
        if ee_pose is None:
            resp.result_status = resp.FAILED
            return resp
        resp.pose = ee_pose
        resp.ee_link = req.ee_frame
        resp.ref_link = req.ref_frame
        resp.result_status = resp.SUCCEEDED
        return resp
