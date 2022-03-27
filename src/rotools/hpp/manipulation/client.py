#!/usr/bin/env python
from __future__ import print_function

import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from roport.srv import *

import rotools.hpp.manipulation.pr2_interface as interface
import rotools.utility.common as common


class HPPManipulationClient(object):
    """This client uses the HPP manipulation interface to connect to the HPP cobra server and provide some
    handy services for manipulation planning.
    """

    def __init__(self, kwargs):
        super(HPPManipulationClient, self).__init__()

        self.interface = interface.HPPManipulationInterface(**kwargs)

        self._srv_execute_manipulation_planning = rospy.Service('execute_manipulation_planning',
                                                                ExecuteManipulationPlanning,
                                                                self.execute_manipulation_planning_handle)
        try:
            js_topic = kwargs['js_topic']
            self._sub_joint_state = rospy.Subscriber(js_topic, JointState, self.update_cb, buff_size=1)
        except KeyError:
            rospy.logwarn("Joint state topic is not provided")

        try:
            odom_topic = kwargs['odom_topic']
            self._sub_odom = rospy.Subscriber(odom_topic, Odometry, self.update_cb, buff_size=1)
        except KeyError:
            rospy.logwarn("Odom topic is not provided")

        try:
            object_pose_topic = kwargs['object_pose_topic']
            self._sub_object_pose = rospy.Subscriber(object_pose_topic, Pose, self.update_cb, buff_size=1)
        except KeyError:
            rospy.logwarn("Object pose topic is not provided")

    def execute_manipulation_planning_handle(self, req):
        resp = ExecuteManipulationPlanningResponse()
        current_base_global_pose = self.interface.get_current_base_global_pose()

        if req.goal_pose_type == req.GLOBAL:
            goal_pose = req.goal_pose
        elif req.goal_pose_type == req.LOCAL_ALIGNED:
            goal_pose = common.local_aligned_pose_to_global_pose(req.goal_pose, current_base_global_pose)
        elif req.goal_pose_type == req.LOCAL:
            goal_pose = common.local_pose_to_global_pose(req.goal_pose, current_base_global_pose)
        else:
            raise NotImplementedError

        # self.interface.set_object_goal_config(goal_pose)

        ok = self.interface.make_plan()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def update_cb(self, msg):
        self.interface.update_current_config(msg)
