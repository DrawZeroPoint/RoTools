#!/usr/bin/env python
from __future__ import print_function

import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, PoseArray

import rotools.xsens.core.interface as interface
from rotools.utility.common import get_param, play_hint_sound


class XsensServer(object):
    """The RoPort server using the RoTools Xsens interface to provide some
    handy services for communicating with the Xsens motion capture devices.
    """

    def __init__(self, kwargs):
        super(XsensServer, self).__init__()
        self.interface = interface.XsensInterface(**kwargs)
        self._enable = False

        # State publisher
        self.all_poses_publisher = rospy.Publisher('/xsens/all_poses', PoseArray, queue_size=1)
        self.body_poses_publisher = rospy.Publisher('/xsens/body_poses', PoseArray, queue_size=1)
        self.left_tcp_publisher = rospy.Publisher('/xsens/left_tcp', PoseStamped, queue_size=1)
        self.right_tcp_publisher = rospy.Publisher('/xsens/right_tcp', PoseStamped, queue_size=1)

        # Publisher switch
        self.srv_pub_switch = rospy.Service('/xsens/enable', SetBool, self.pub_switch_handle)

        rate = get_param("publish_rate")
        self.all_poses_msg_timer = rospy.Timer(rospy.Duration(1.0 / rate), self.all_poses_msg_handle)

    def all_poses_msg_handle(self, event):
        if not self._enable:
            return
        ok, all_poses = self.interface.get_all_poses()
        if ok:
            self.all_poses_publisher.publish(all_poses)
            body_poses, left_tcp, right_tcp = self.interface.get_body_pose_array_msg(all_poses)
            self.body_poses_publisher.publish(body_poses)
            self.left_tcp_publisher.publish(left_tcp)
            self.right_tcp_publisher.publish(right_tcp)

    def pub_switch_handle(self, req):
        if req.data:
            self._enable = True
            msg = 'Xsens stream receiving enabled'
        else:
            self._enable = False
            msg = 'Xsens stream receiving disabled'
        play_hint_sound(req.data)
        return SetBoolResponse(True, msg)
