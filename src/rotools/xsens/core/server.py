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

    def __init__(self, kwargs):
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

        self.pub_prop = kwargs['prop']
        # Prop pose publisher
        self.prop_1_publisher = rospy.Publisher('/xsens/prop_1', PoseStamped, queue_size=1)

        self.pub_detail = kwargs['detail']
        # Detail publisher
        self.left_shoulder_publisher = rospy.Publisher('/xsens/left_shoulder', PoseStamped, queue_size=1)
        self.right_shoulder_publisher = rospy.Publisher('/xsens/right_shoulder', PoseStamped, queue_size=1)
        self.left_upper_arm_publisher = rospy.Publisher('/xsens/left_upper_arm', PoseStamped, queue_size=1)
        self.right_upper_arm_publisher = rospy.Publisher('/xsens/right_upper_arm', PoseStamped, queue_size=1)
        self.left_forearm_publisher = rospy.Publisher('/xsens/left_forearm', PoseStamped, queue_size=1)
        self.right_forearm_publisher = rospy.Publisher('/xsens/right_forearm', PoseStamped, queue_size=1)

        # Joint states publishers
        self.left_hand_publisher = rospy.Publisher('/xsens/left_hand_js', JointState, queue_size=1)
        self.right_hand_publisher = rospy.Publisher('/xsens/right_hand_js', JointState, queue_size=1)

        # Tracked object publishers
        self.object_pose_publisher = rospy.Publisher('/xsens/object', PoseStamped, queue_size=1)

        rate = kwargs['rate']
        self.all_poses_msg_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / rate), self.all_poses_msg_handle)

    def all_poses_msg_handle(self, event):
        if not self.enabled:
            return
        ok, all_poses = self.interface.get_all_poses()
        if ok:
            body_poses, left_tcp, right_tcp, left_sole, right_sole, left_shoulder, left_upper_arm, left_forearm,\
                right_shoulder, right_upper_arm, right_forearm = self.interface.get_body_pose_array_msg(all_poses)

            if self.interface.object_poses is not None:
                # TODO dzp: for now we only publish the first object pose
                self.object_pose_publisher.publish(self.interface.first_object_pose)

            self.all_poses_publisher.publish(all_poses)
            self.body_poses_publisher.publish(body_poses)
            self.left_tcp_publisher.publish(left_tcp)
            self.right_tcp_publisher.publish(right_tcp)
            self.left_sole_publisher.publish(left_sole)
            self.right_sole_publisher.publish(right_sole)

            if self.pub_detail:
                self.left_shoulder_publisher.publish(left_shoulder)
                self.right_shoulder_publisher.publish(right_shoulder)
                self.left_upper_arm_publisher.publish(left_upper_arm)
                self.right_upper_arm_publisher.publish(right_upper_arm)
                self.left_forearm_publisher.publish(left_forearm)
                self.right_forearm_publisher.publish(right_forearm)

            prop_1 = self.interface.get_prop_msgs(all_poses)
            if prop_1 is not None and self.pub_prop:
                self.prop_1_publisher.publish(prop_1)

            left_hand_js, right_hand_js = self.interface.get_hand_joint_states(all_poses)
            if left_hand_js is not None:
                self.left_hand_publisher.publish(left_hand_js)
            if right_hand_js is not None:
                self.right_hand_publisher.publish(right_hand_js)

    def pub_switch_handle(self, req):
        if req.data:
            self.set_status(True)
            msg = 'Xsens stream receiving enabled'
        else:
            self.set_status(False)
            msg = 'Xsens stream receiving disabled'
        play_hint_sound(req.data)
        return SetBoolResponse(True, msg)
