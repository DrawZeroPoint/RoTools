#!/usr/bin/env python
from __future__ import print_function

import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState

import rotools.xsens.core.interface as interface

from rotools.utility.emergency_stop import EStop
from rotools.utility.common import play_hint_sound


class XsensServer(EStop):
    """The RoPort server using the RoTools Xsens interface to provide some
    handy services for communicating with the Xsens motion capture devices.
    """

    def __init__(self, kwargs, rate):
        super(XsensServer, self).__init__()

        # Publisher switch
        self.srv_pub_switch = rospy.Service('/xsens/enable', SetBool, self.pub_switch_handle)

        self.interface = interface.XsensInterface(**kwargs)

        # Cartesian pose publishers
        self.all_poses_publisher = rospy.Publisher('/xsens/all_poses', PoseArray, queue_size=1)
        self.body_poses_publisher = rospy.Publisher('/xsens/body_poses', PoseArray, queue_size=1)
        self.left_tcp_publisher = rospy.Publisher('/xsens/left_tcp', PoseStamped, queue_size=1)
        self.right_tcp_publisher = rospy.Publisher('/xsens/right_tcp', PoseStamped, queue_size=1)
        self.left_sole_publisher = rospy.Publisher('/xsens/left_sole', PoseStamped, queue_size=1)
        self.right_sole_publisher = rospy.Publisher('/xsens/right_sole', PoseStamped, queue_size=1)

        # Joint states publishers
        self.left_hand_publisher = rospy.Publisher('/xsens/left_hand_js', JointState, queue_size=1)
        self.right_hand_publisher = rospy.Publisher('/xsens/right_hand_js', JointState, queue_size=1)

        self.all_poses_msg_timer = rospy.Timer(rospy.Duration(1.0 / rate), self.all_poses_msg_handle)

    def all_poses_msg_handle(self, event):
        if not self.enable:
            return
        ok, all_poses = self.interface.get_all_poses()
        if ok:
            self.all_poses_publisher.publish(all_poses)
            body_poses, left_tcp, right_tcp, left_sole, right_sole = self.interface.get_body_pose_array_msg(all_poses)
            self.body_poses_publisher.publish(body_poses)
            self.left_tcp_publisher.publish(left_tcp)
            self.right_tcp_publisher.publish(right_tcp)
            self.left_sole_publisher.publish(left_sole)
            self.right_sole_publisher.publish(right_sole)
            left_hand_js, right_hand_js = self.interface.get_hand_joint_states(all_poses)
            if left_hand_js is not None:
                self.left_hand_publisher.publish(left_hand_js)
            if right_hand_js is not None:
                self.right_hand_publisher.publish(right_hand_js)

    def pub_switch_handle(self, req):
        if req.data:
            self.enable = True
            msg = 'Xsens stream receiving enabled'
        else:
            self.enable = False
            msg = 'Xsens stream receiving disabled'
        play_hint_sound(req.data)
        return SetBoolResponse(True, msg)
