#!/usr/bin/env python
from __future__ import print_function

import rospy

from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState

import rotools.xsens.core.interface as interface

from rotools.utility.emergency_stop import EStop
from rotools.utility.common import print_warn


class XsensServer(EStop):
    """The RoPort server using the RoTools Xsens interface to provide some
    handy services for communicating with the Xsens motion capture devices.
    """

    def __init__(self, kwargs):
        super(XsensServer, self).__init__()

        # Publisher switch
        self.srv_pub_switch = rospy.Service(
            "/xsens/enable", SetBool, self.pub_switch_handle
        )
        print_warn(
            'Use [rosservice call /xsens/enable "data: true"] to enable receiving XSens data.'
        )

        self.interface = interface.XsensInterface(**kwargs)

        # Cartesian pose publishers
        self.all_poses_publisher = rospy.Publisher(
            "/xsens/all_poses", PoseArray, queue_size=1
        )
        self.body_poses_publisher = rospy.Publisher(
            "/xsens/body_poses", PoseArray, queue_size=1
        )
        self.pose_array_publishers = [
            self.all_poses_publisher,
            self.body_poses_publisher,
        ]

        self.base_pose_publisher = rospy.Publisher(
            "/xsens/base", PoseStamped, queue_size=1
        )
        self.left_tcp_publisher = rospy.Publisher(
            "/xsens/left_tcp", PoseStamped, queue_size=1
        )
        self.right_tcp_publisher = rospy.Publisher(
            "/xsens/right_tcp", PoseStamped, queue_size=1
        )
        self.left_sole_publisher = rospy.Publisher(
            "/xsens/left_sole", PoseStamped, queue_size=1
        )
        self.right_sole_publisher = rospy.Publisher(
            "/xsens/right_sole", PoseStamped, queue_size=1
        )
        self.head_publisher = rospy.Publisher("/xsens/head", PoseStamped, queue_size=1)
        self.core_poses_publishers = (
            self.base_pose_publisher,
            self.left_tcp_publisher,
            self.right_tcp_publisher,
            self.left_sole_publisher,
            self.right_sole_publisher,
            self.head_publisher,
        )

        self.pub_prop = kwargs["prop"]
        # Prop pose publisher
        self.prop_1_publisher = rospy.Publisher(
            "/xsens/prop_1", PoseStamped, queue_size=1
        )

        # Customized poses publishers
        self.customized_poses = rospy.get_param("~customized_poses", [])

        if self.customized_poses:
            rospy.loginfo("Defined customized poses: {}".format(self.customized_poses))
            self.customized_poses_publishers = []
            for i, entity in enumerate(self.customized_poses):
                try:
                    _, _, topic_id = entity
                    publisher = rospy.Publisher(topic_id, PoseStamped, queue_size=1)
                    self.customized_poses_publishers.append(publisher)
                except ValueError as e:
                    rospy.logerr("Entity {}: {}".format(i, e))

        # Hand joint states publishers
        self.left_hand_publisher = rospy.Publisher(
            "/xsens/left_hand_js", JointState, queue_size=1
        )
        self.right_hand_publisher = rospy.Publisher(
            "/xsens/right_hand_js", JointState, queue_size=1
        )

        # Tracked object publishers
        self.object_pose_publisher = rospy.Publisher(
            "/xsens/object", PoseStamped, queue_size=1
        )

        rate = kwargs["rate"]
        self.all_poses_msg_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / rate), self.all_poses_msg_handle
        )

    def all_poses_msg_handle(self, event):
        """ """
        if not self.enabled:
            return
        if not self.interface.get_datagram():
            return
        if self.interface.header.is_object:
            # Object pose (if exist) is always published together with body poses
            return

        poses, core_poses = self.interface.get_body_poses()

        for pub, p in zip(self.pose_array_publishers, poses):
            pub.publish(p)

        for pub, p in zip(self.core_poses_publishers, core_poses):
            pub.publish(p)

        self.pub_customized_poses()

        if self.interface.first_object_pose is not None:
            self.object_pose_publisher.publish(self.interface.first_object_pose)
            self.interface.object_poses = None

        prop_1 = self.interface.get_prop_msgs()
        if prop_1 is not None and self.pub_prop:
            self.prop_1_publisher.publish(prop_1)

        left_hand_js, right_hand_js = self.interface.get_hand_joint_states()
        if left_hand_js is not None:
            self.left_hand_publisher.publish(left_hand_js)
        if right_hand_js is not None:
            self.right_hand_publisher.publish(right_hand_js)

    def pub_customized_poses(self):
        for i, customized_pose in enumerate(self.customized_poses):
            base_frame, distal_frame, _ = customized_pose
            pose_msg = self.interface.get_transform(base_frame, distal_frame)
            if pose_msg is not None:
                self.customized_poses_publishers[i].publish(pose_msg)

    def pub_switch_handle(self, req):
        if req.data:
            self.set_status(True)
            msg = "Xsens stream receiving enabled"
        else:
            self.set_status(False)
            msg = "Xsens stream receiving disabled"
        return SetBoolResponse(True, msg)
