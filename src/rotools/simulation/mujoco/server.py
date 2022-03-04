#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import rospy

from sensor_msgs.msg import JointState

import rotools.simulation.mujoco.interface as interface


class MuJoCoServer(object):
    """The RoPort server using the RoTools MuJoCoServer interface to provide some
    handy services for controlling serial robot arm.
    """

    def __init__(self, kwargs):
        """

        Args:
            kwargs:
        """
        super(MuJoCoServer, self).__init__()

        self.interface = interface.MuJoCoInterface(**kwargs)
        self.interface.start()

        command_topic_id = kwargs['command_topic_id']
        self.joint_command_subscriber = rospy.Subscriber(command_topic_id, JointState, self.joint_command_cb)

        state_topic_id = kwargs['state_topic_id']
        self.joint_state_publisher = rospy.Publisher(state_topic_id, JointState, queue_size=1)
        rate = kwargs['rate']
        self.publish_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.joint_state_handle)

    def joint_state_handle(self, event):
        joint_state_msg = self.interface.get_joint_states()
        if joint_state_msg:
            self.joint_state_publisher.publish(joint_state_msg)

    def joint_command_cb(self, cmd):
        if not cmd.name:
            rospy.logwarn_throttle(3, 'Joint command contains no name')
        if len(cmd.position) != self.interface.actuator_num:
            rospy.logerr('Joint command size {} and actuator number {} mismatch'.format(
                len(cmd.position), self.interface.actuator_num))
            return

        self.interface.set_joint_commands(cmd)
