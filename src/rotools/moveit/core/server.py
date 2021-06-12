#!/usr/bin/env python
from __future__ import print_function

import rospy

from roport.srv import *

import rotools.moveit.core.interface as interface


class MoveItServer(object):
    """The RoPort server using the RoTools MoveIt interface to provide some
    handy services for controlling serial robot arm.
    """

    def __init__(self, kwargs):

        super(MoveItServer, self).__init__()

        self.interface = interface.MoveGroupInterface(**kwargs)

        # MoveIt information getters
        self._srv_get_names = rospy.Service('get_all_group_names', GetAllNames, self.get_all_names_handle)
        self._srv_get_act_names = rospy.Service('get_active_group_names', GetAllNames, self.get_active_names_handle)
        self._srv_get_pose = rospy.Service('get_group_pose', GetGroupPose, self.get_pose_handle)
        self._srv_get_js = rospy.Service('get_group_joint_states', GetGroupJointStates, self.get_js_handle)

        # Robot movement executors
        # For a group
        self._srv_group_position = rospy.Service(
            'execute_group_position', ExecuteGroupPosition, self.group_position_handle
        )
        self._srv_group_shift = rospy.Service('execute_group_shift', ExecuteGroupShift, self.group_shift_handle)
        self._srv_group_pose = rospy.Service('execute_group_pose', ExecuteGroupPose, self.group_pose_handle)
        self._srv_group_js = rospy.Service('execute_group_joint_states', ExecuteGroupJointStates, self.group_js_handle)
        self._srv_group_home = rospy.Service(
            'execute_group_named_states', ExecuteGroupNamedStates, self.group_named_states_handle
        )
        self._srv_group_plan = rospy.Service('execute_group_plan', ExecuteGroupPlan, self.group_plan_handle)

        # For multiple groups
        self._srv_all_pose = rospy.Service('execute_all_poses_py', ExecuteAllPoses, self.all_poses_handle)

        # Planning scene helpers
        self._srv_add_box = rospy.Service('execute_add_collision_box', ExecuteAddCollisionBox, self.add_box_handle)
        self._srv_add_plane = rospy.Service(
            'execute_add_collision_plane', ExecuteAddCollisionPlane, self.add_plane_handle
        )
        self._srv_attach_box = rospy.Service(
            'execute_attach_collision_box', ExecuteAttachCollisionBox, self.attach_box_handle
        )
        self._srv_detach_obj = rospy.Service(
            'execute_detach_collision', ExecuteDetachCollision, self.detach_object_handle
        )
        self._srv_remove_obj = rospy.Service(
            'execute_remove_collision', ExecuteRemoveCollision, self.remove_object_handle
        )

        # General utilities
        self._srv_get_prepare_pose = rospy.Service('get_prepare_pose', GetPreparePose, self.prepare_pose_handle)
        self._srv_get_transformed_pose = rospy.Service(
            'get_transformed_pose', GetTransformedPose, self.transformed_pose_handle
        )

    def get_all_names_handle(self, req):
        resp = GetAllNamesResponse()
        try:
            names = self.interface.get_all_group_names()
            resp.result_status = resp.SUCCEEDED
            resp.group_names = names
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_active_names_handle(self, req):
        resp = GetAllNamesResponse()
        try:
            names = self.interface.get_active_group_names()
            resp.result_status = resp.SUCCEEDED
            resp.group_names = names
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_pose_handle(self, req):
        resp = GetGroupPoseResponse()
        try:
            resp.pose = self.interface.get_current_pose_of_group(req.group_name)
            resp.ee_link, resp.ref_link = self.interface.get_frame_of_group(req.group_name)
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def get_js_handle(self, req):
        resp = GetGroupJointStatesResponse()
        try:
            joint_names = self.interface.get_active_joint_names_of_group(req.group_name)
            joint_states = self.interface.get_joint_states_of_group(req.group_name, req.result_type)
            resp.joint_names = joint_names
            resp.joint_states = joint_states
            resp.result_status = resp.SUCCEEDED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def group_shift_handle(self, req):
        ok = self.interface.group_shift(req.group_name, req.axis, req.goal)
        resp = ExecuteGroupShiftResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_position_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01
        if req.goal_type == req.BASE_ABS:
            ok = self.interface.group_go_to_global_base_position(req.group_name, req.goal, req.tolerance)
        elif req.goal_type == req.BASE_REL:
            ok = False
        else:
            ok = self.interface.group_go_to_eef_position(req.group_name, req.goal, req.tolerance)
        resp = ExecuteGroupPositionResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_pose_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01
        if req.goal_type == req.BASE_ABS:
            ok = self.interface.group_goto_pose_goal_abs(req.group_name, req.goal, req.tolerance, req.constraint)
        elif req.goal_type == req.BASE_REL:
            ok = self.interface.group_goto_pose_goal_rel(req.group_name, req.goal, req.tolerance, req.constraint)
        elif req.goal_type == req.EEF:
            ok = self.interface.group_goto_pose_goal_eef(req.group_name, req.goal, req.tolerance, req.constraint)
        else:
            rospy.logerr('Unknown goal type for group pose: {}'.format(req.goal_type))
            ok = False
        resp = ExecuteGroupPoseResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_js_handle(self, req):
        if not req.tolerance:
            req.tolerance = 0.01
        ok = self.interface.group_go_to_joint_states(req.group_name, req.goal, req.tolerance)
        resp = ExecuteGroupJointStatesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_named_states_handle(self, req):
        ok = self.interface.group_go_to_named_states(req.group_name, req.state_name)
        resp = ExecuteGroupJointStatesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def group_plan_handle(self, req):
        if req.is_absolute:
            plan = self.interface.build_absolute_path_for_group(req.group_name, req.poses, req.stamp,
                                                                not req.allow_collision)
        else:
            plan = self.interface.build_relative_path_for_group(req.group_name, req.poses, req.stamp,
                                                                not req.allow_collision)

        ok = self.interface.execute_plan_for_group(req.group_name, plan)
        resp = ExecuteGroupPlanResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def all_poses_handle(self, req):
        if req.goal_type == req.BASE_ABS:
            ok = self.interface.all_goto_pose_goals_abs(req.group_names, req.goals, req.tolerance,
                                                        req.stamps, req.allow_collision)
        elif req.goal_type == req.BASE_REL:
            ok = self.interface.all_goto_pose_goals_rel(req.group_names, req.goals, req.tolerance,
                                                        req.stamps, req.allow_collision)
        elif req.goal_type == req.EEF:
            ok = self.interface.all_goto_pose_goals_eef(req.group_names, req.goals, req.tolerance,
                                                        req.stamps, req.allow_collision)
        else:
            raise NotImplementedError
        resp = ExecuteAllPosesResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    # def all_plans_handle(self, req):
    #     if req.is_absolute:
    #         all_plans = self.interface.build_absolute_paths_for_all(req.all_poses, req.stamps, not req.allow_collision)
    #     else:
    #         all_plans = self.interface.build_relative_paths_for_all(req.all_poses, req.stamps, not req.allow_collision)
    #     ok = self.interface.execute_plans_for_all(all_plans)
    #     resp = ExecuteAllPlansResponse()
    #     resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
    #     return resp

    def add_box_handle(self, req):
        ok = self.interface.add_box(req.group_name, req.box_name, req.box_pose,
                                    req.box_size, req.is_absolute, req.auto_suffix)
        resp = ExecuteAddCollisionBoxResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def attach_box_handle(self, req):
        ok = self.interface.attach_box(req.group_name, req.eef_group_name, req.box_name, req.box_pose, req.box_size)
        resp = ExecuteAttachCollisionBoxResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def add_plane_handle(self, req):
        ok = self.interface.add_plane(req.group_name, req.plane_name, req.plane_pose, req.plane_normal)
        resp = ExecuteAddCollisionPlaneResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def detach_object_handle(self, req):
        ok = self.interface.detach_object(req.group_name, req.obj_name)
        resp = ExecuteDetachCollisionResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def remove_object_handle(self, req):
        ok = self.interface.remove_object(req.obj_name, req.is_exact)
        resp = ExecuteRemoveCollisionResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        return resp

    def prepare_pose_handle(self, req):
        ok, pose = self.interface.get_prepare_pose(req.pose, req.is_absolute, req.shift)
        resp = GetPreparePoseResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        resp.pre_pose = pose
        return resp

    def transformed_pose_handle(self, req):
        ok, pose = self.interface.get_transformed_pose(req.pose, req.source_frame, req.target_frame)
        resp = GetTransformedPoseResponse()
        resp.result_status = resp.SUCCEEDED if ok else resp.FAILED
        resp.trans_pose = pose
        return resp
