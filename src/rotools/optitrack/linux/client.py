#!/usr/bin/env python3

import socket

import rospy

import nav_msgs
import geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import re


def data_process(data):
    position = re.findall(r'Position\s*:\s*(.*)', data)[0]
    orientation = re.findall(r'Orientation\s*:\s*(.*)', data)[0]
    return eval(position), eval(orientation)


class OptiTrackClient(object):
    """Class for receiving rigid body tracking information from OptiTrack device."""

    def __init__(self, kwargs):
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._client.connect((kwargs['ip'], kwargs['port']))
        rospy.loginfo("Connected to socket: {}:{}".format(kwargs['ip'], kwargs['port']))

        self.timer = rospy.Timer(rospy.Duration.from_sec(1. / kwargs['rate']), self._socket_cb)

        self._advertise_dict = {}

        if kwargs['pose_topic'] is not None:
            topics = kwargs['pose_topic']
            self.register_topic(topics, geometry_msgs.msg.Pose)

        if kwargs['odom_topic'] is not None:
            topics = kwargs['odom_topic']
            self.register_topic(topics, nav_msgs.msg.Odometry)

    def register_topic(self, topics, msg_type):
        if isinstance(topics, str):
            publisher = self.create_publisher(topics, msg_type)
            self._advertise_dict[topics] = [msg_type, publisher]
        elif isinstance(topics, list) or isinstance(topics, tuple):
            for topic in topics:
                publisher = self.create_publisher(topic, msg_type)
                self._advertise_dict[topic] = [msg_type, publisher]
        else:
            raise NotImplementedError

    @staticmethod
    def create_publisher(topic_id, msg_type):
        return rospy.Publisher(topic_id, msg_type, queue_size=1)

    def _socket_cb(self, event):
        utf_data = self._client.recv(1024).decode('utf-8')
        position, orientation = data_process(utf_data)
        position = Point(*position)
        orientation = Quaternion(*orientation)
        # print('position:')
        print(position)
        # print('orientation:')
        # print(orientation)
        for _, entity in self._advertise_dict.items():
            msg_type, publisher = entity
            if msg_type is geometry_msgs.msg.Pose:
                msg = Pose()
                msg.position = position
                msg.orientation = orientation
            elif msg_type is nav_msgs.msg.Odometry:
                msg = Odometry()
                msg.pose.pose.position = position
                msg.pose.pose.orientation = orientation
            else:
                raise NotImplementedError
            publisher.publish(msg)
        self._client.send('ok'.encode('utf-8'))



def test():
    rospy.init_node('rigid_body_pose_publisher')
    args_dict = {'ip':'192.168.13.118', 'port':6688, 'rate':500, 'pose_topic':None,'odom_topic':'odom'}
    client = OptiTrackClient(args_dict)


if __name__=='__main__':
    test()
    while True:
        pass