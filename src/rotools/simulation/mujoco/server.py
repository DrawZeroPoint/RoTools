#!/usr/bin/env python
from __future__ import print_function

import rospy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import rotools.simulation.mujoco.interface as interface
from rotools.utility.robotics import mecanum_base_get_wheel_velocities


class MuJoCoServer(object):
    """The RoPort server using the RoTools MuJoCoServer interface to provide some
    handy services for controlling serial robot arm.
    """

    def __init__(self, kwargs):
        """Initialize the MuJoCoServer.

        Args:
            kwargs: dict Configurations.
        """
        super(MuJoCoServer, self).__init__()

        self.interface = interface.MuJoCoInterface(**kwargs)
        self.interface.start()

        # Received msgs
        joint_command_topic_id = kwargs['joint_command_topic_id']
        self.joint_command_subscriber = rospy.Subscriber(joint_command_topic_id, JointState, self.joint_command_cb)

        if 'base_command_topic_id' in kwargs.keys():
            base_command_topic_id = kwargs['base_command_topic_id']
            self.base_command_subscriber = rospy.Subscriber(base_command_topic_id, Twist, self.base_command_cb)

        # Published msgs
        rate = kwargs['publish_rate']

        joint_state_topic_id = kwargs['joint_state_topic_id']
        self.joint_state_publisher = rospy.Publisher(joint_state_topic_id, JointState, queue_size=1)
        self.js_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.joint_state_handle)

        if 'odom_topic_id' in kwargs.keys():
            odom_topic_id = kwargs['odom_topic_id']
            self.odom_publisher = rospy.Publisher(odom_topic_id, Odometry, queue_size=1)
            self.odom_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.odom_handle)

        # Get robot base params if available
        self.wheel_radius = kwargs['wheel_radius'] if 'wheel_radius' in kwargs.keys() else None
        self.base_width = kwargs['base_width'] if 'base_width' in kwargs.keys() else None
        self.base_length = kwargs['base_length'] if 'base_length' in kwargs.keys() else None

    def joint_state_handle(self, _):
        joint_state_msg = self.interface.get_joint_states()
        if joint_state_msg:
            self.joint_state_publisher.publish(joint_state_msg)

    def odom_handle(self, _):
        odom_msg = self.interface.get_odom()
        if odom_msg:
            self.odom_publisher.publish(odom_msg)

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