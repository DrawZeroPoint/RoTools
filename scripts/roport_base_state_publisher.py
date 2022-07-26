#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from rotools.utility.common import get_param


class BaseStatePublisher(object):
    """Given the odometry msg from the global reference frame to the robot base frame,
    convert the transform to /tf msg.
    """

    def __init__(self):
        super(BaseStatePublisher, self).__init__()

        odom_topic = get_param("odom_topic")
        self.global_reference_frame = get_param("global_reference_frame")
        self.robot_base_frame = get_param("robot_base_frame")
        rospy.loginfo(
            "Publishing /tf from '{}' to '{}' with odom topic {}".format(
                self.global_reference_frame, self.robot_base_frame, odom_topic
            )
        )
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self.odom_cb, queue_size=1
        )

    def odom_cb(self, msg):
        assert isinstance(msg, Odometry)
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.global_reference_frame
        t.child_frame_id = self.robot_base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        br.sendTransform(t)


if __name__ == "__main__":
    try:
        rospy.init_node("roport_base_state_publisher")
        bsp = BaseStatePublisher()
        rospy.loginfo("RoPort: Base State Publisher ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
