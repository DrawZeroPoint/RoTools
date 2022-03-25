#!/usr/bin/env python
from __future__ import print_function

import nav_msgs.msg
import rospy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from roport.srv import *

import rotools.snapshot.core.interface as interface


class SnapshotServer(object):
    """This server uses the snapshot interface to provide some
    handy services for recording data from ROS topics.
    """

    def __init__(self, kwargs):
        super(SnapshotServer, self).__init__()

        self._subscribers = []
        self._msg_dict = {}

        kwargs['js_topics'] = self.register_topics(kwargs['js_topics'], sensor_msgs.msg.JointState)
        kwargs['odom_topics'] = self.register_topics(kwargs['odom_topics'], nav_msgs.msg.Odometry)

        self.interface = interface.SnapshotInterface(**kwargs)

        self._srv_save_image = rospy.Service('save_image', SaveImage, self.save_image_handle)
        self._srv_save_joint_state = rospy.Service('save_joint_state', SaveJointState, self.save_joint_state_handle)
        self._srv_save_odom = rospy.Service('save_odom', SaveOdometry, self.save_odom_handle)

    def register_topics(self, topics, topic_type):
        """Register subscribers for cared topics.

        Args:
            topics: list[str]/tuple(str) Topic names.
            topic_type: class ROS msg type.

        Returns:
            A list of valid topic names.
        """
        if isinstance(topics, list) or isinstance(topics, tuple):
            valid_topics = []
            for topic in topics:
                # TODO if rospy.topics.is_topic()
                subscriber = rospy.Subscriber(topic, topic_type, self.msg_cb, topic)
                self._subscribers.append(subscriber)
                rospy.loginfo("Registered topic {} of type {}".format(topic, topic_type))
                valid_topics.append(topic)
            return valid_topics
        else:
            return None

    def save_image_handle(self, req):
        pass

    def save_joint_state_handle(self, req):
        resp = SaveJointStateResponse()
        if req.topic in self._msg_dict:
            msg = self._msg_dict[req.topic]
            ok = self.interface.save_joint_state_msg(req.topic, msg, req.position_only, req.tag)
            resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        else:
            rospy.logwarn("Joint state topic {} has not been recorded".format(req.topic))
            resp.result_status = resp.FAILED
        return resp

    def save_odom_handle(self, req):
        resp = SaveOdometryResponse()
        if req.topic in self._msg_dict:
            msg = self._msg_dict[req.topic]
            ok = self.interface.save_odom_msg(req.topic, msg, req.tag)
            resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        else:
            rospy.logwarn("Odometry topic {} has not been recorded".format(req.topic))
            resp.result_status = resp.FAILED
        return resp

    def msg_cb(self, msg, arg):
        self._msg_dict[arg] = msg
