#!/usr/bin/env python
from __future__ import print_function

import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from roport.srv import *

import rotools.hpp.manipulation.interface as interface
import rotools.utility.common as common


class HPPManipulationClient(object):
    """This client uses the HPP manipulation interface to connect to the HPP cobra server and provide some
    handy services for manipulation planning.
    """

    def __init__(self, kwargs):
        super(HPPManipulationClient, self).__init__()

        self.interface = interface.HPPManipulationInterface(**kwargs)

        self._srv_execute_path_planning = rospy.Service('execute_path_planning', ExecutePathPlanning,
                                                        self.execute_path_planning_handle)

        self._srv_execute_manipulation_planning = rospy.Service('execute_manipulation_planning',
                                                                ExecuteManipulationPlanning,
                                                                self.execute_manipulation_planning_handle)
        try:
            js_topic = kwargs['joint_state_topic']
            self._sub_joint_state = rospy.Subscriber(js_topic, JointState, self.update_cb, buff_size=1)
        except KeyError:
            rospy.logwarn("Joint state topic is not provided")

        try:
            odom_topic = kwargs['odom_topic']
            self._sub_odom = rospy.Subscriber(odom_topic, Odometry, self.odom_update_cb, buff_size=1)
        except KeyError:
            rospy.logwarn("Odom topic is not provided")

        try:
            object_pose_topic = kwargs['object_pose_topic']
            self._sub_object_pose = rospy.Subscriber(object_pose_topic, Pose, self.update_cb, buff_size=1)
        except KeyError:
            rospy.logwarn("Object pose topic is not provided")

    def execute_path_planning_handle(self, req):
        resp = ExecutePathPlanningResponse()
        # req = ExecutePathPlanningRequest()
        base_current_global_pose = self.interface.get_current_base_global_pose()
        base_goal_pose = self._to_global_pose(req.base_goal_pose, req.base_goal_type, base_current_global_pose)
        ok = self.interface.approach(base_goal_pose, req.joint_goal_state, req.base_pos_tolerance,
                                     req.base_ori_tolerance)
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def execute_manipulation_planning_handle(self, req):
        resp = ExecuteManipulationPlanningResponse()
        base_current_global_pose = self.interface.get_current_base_global_pose()
        object_goal_pose = self._to_global_pose(req.object_goal_pose, req.object_goal_pose_type,
                                                base_current_global_pose)
        # base_goal_pose = self._to_global_pose(req.base_goal_pose, req.base_goal_pose_type, base_current_global_pose)
        ok = self.interface.grasp(req.joint_goal_state, object_goal_pose, req.object_pos_tolerance,
                                  req.object_ori_tolerance)
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def _to_global_pose(self, pose, pose_type, reference_pose):
        if pose_type == 0:
            return pose
        elif pose_type == 1:
            return common.local_aligned_pose_to_global_pose(pose, reference_pose)
        elif pose_type == 2:
            return common.local_pose_to_global_pose(pose, reference_pose)
        else:
            raise NotImplementedError

    def update_cb(self, msg):
        self.interface.update_current_config(msg)

    def odom_update_cb(self, msg):
        self.interface.update_current_config(msg)
