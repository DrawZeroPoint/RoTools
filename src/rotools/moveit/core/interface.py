from __future__ import print_function

import sys
import math
import numpy as np

try:
    import rospy
    import tf2_ros
    import tf2_geometry_msgs  # import this is mandatory to use PoseStamped msg

    import moveit_commander

    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import control_msgs.msg as ControlMsg
    import trajectory_msgs.msg as TrajectoryMsg
    import std_msgs.msg as StdMsg
    import sensor_msgs.msg as SensorMsg
except ImportError:
    pass

from rotools.utility import common, transform


class MoveGroupInterface(object):

    def __init__(
            self,
            robot_description,
            ns,
            group_names,
            ref_frames=None,
            ee_links=None,
    ):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.commander = moveit_commander.RobotCommander(robot_description, ns)

        if not isinstance(group_names, list) and not isinstance(group_names, tuple):
            raise TypeError('group_names should be list or tuple, but got {}'.format(type(group_names)))

        self.group_names = group_names
        self.group_num = len(self.group_names)
        assert self.group_num >= 1

        # Get a set G of all groups of the robot, the used groups G' could be a subset of G
        all_group_names = self.commander.get_group_names()
        for name in self.group_names:
            assert name in all_group_names, 'Group name {} does not exist'.format(name)

        self.move_groups = []
        for name in self.group_names:
            g = moveit_commander.MoveGroupCommander(name)
            # For pilz planner
            # g.set_planner_id('LIN')
            # g.set_max_velocity_scaling_factor(0.3)
            # g.set_max_acceleration_scaling_factor(0.3)
            self.move_groups.append(g)

        if not ref_frames:
            self.ref_frames = []
            for group in self.move_groups:
                self.ref_frames.append(group.get_planning_frame())
        else:
            assert len(ref_frames) == self.group_num
            self.ref_frames = ref_frames
            for i, group in enumerate(self.move_groups):
                group.set_pose_reference_frame(ref_frames[i])

        if not ee_links:
            self.ee_links = []
            for group in self.move_groups:
                self.ee_links.append(group.get_end_effector_link())
        else:
            assert len(ee_links) == self.group_num
            self.ee_links = ee_links
            for i, group in enumerate(self.move_groups):
                group.set_end_effector_link(self.ee_links[i])

        self.scene = moveit_commander.PlanningSceneInterface(ns=ns)
        self._obj_suffix = 0

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print(self.commander.get_current_state())

        # TF related handles
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def get_all_group_names(self):
        return self.commander.get_group_names()

    def get_active_group_names(self):
        return self.group_names

    def _get_group_id(self, group_name):
        for i, name in enumerate(self.group_names):
            if name == group_name:
                return i
        return None

    def _get_group_by_name(self, group_name):
        for i, name in enumerate(self.group_names):
            if name == group_name:
                return self.move_groups[i]
        raise IndexError('The group name {} is not in the known names {}'.format(group_name, self.group_names))

    def get_active_joint_names_of_all_groups(self):
        ret = []
        for group in self.move_groups:
            ret.append(group.get_active_joints())
        return ret

    def get_active_joint_names_of_group(self, group_name):
        group = self._get_group_by_name(group_name)
        return group.get_active_joints()

    def get_joint_states_of_all_groups(self):
        """Get joint states of all move groups

        :return: List[List[float]]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_joint_values())
        return ret

    def get_joint_states_of_group(self, group_name, result_type='rad'):
        group = self._get_group_by_name(group_name)
        j_values = group.get_current_joint_values()
        if result_type == 'deg' or result_type == 'd':
            j_values = np.rad2deg(j_values)
        return j_values

    def get_current_poses_of_all_groups(self):
        """Get the eef pose in ROS format.

        :return: List[PoseStamped]
        """
        ret = []
        for group in self.move_groups:
            ret.append(group.get_current_pose())
        return ret

    def get_current_pose_of_group(self, group_name):
        group = self._get_group_by_name(group_name)
        return group.get_current_pose().pose

    def get_current_position_of_group(self, group_name):
        current_pose = self.get_current_pose_of_group(group_name)
        return current_pose.position

    def get_frame_of_group(self, group_name):
        group_id = self._get_group_id(group_name)
        assert group_id is not None
        return self.ee_links[group_id], self.ref_frames[group_id]

    @staticmethod
    def get_prepare_pose(pose, is_absolute, shift):
        """Given a target pose of the end-effector, get the prepare pose
         for the robot to make the eef move to the target pose.

        :param pose: target pose of the end-effector in base reference frame
        :param is_absolute: if true, the shift vector is wrt the base frame, else wrt the target pose frame
        :param shift: shift vector pointing from the target pose to prepare pose, in meters
        """
        assert isinstance(pose, GeometryMsg.Pose), print("Pose format error")
        assert isinstance(shift, GeometryMsg.Point), print("Shift format error")
        sd_shift = np.array([shift.x, shift.y, shift.z])
        if not is_absolute:
            q = pose.orientation
            m = transform.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            sd_shift = np.dot(m, sd_shift)
        return True, common.offset_ros_pose(pose, sd_shift)

    def get_transformed_pose(self, pose, source_frame, target_frame):
        assert isinstance(pose, GeometryMsg.Pose), print("Pose format error")
        pose_stamped = GeometryMsg.PoseStamped()
        pose_stamped.pose = pose
        # pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = source_frame
        try:
            transformed_pose = self._tf_buffer.transform(pose_stamped, target_frame)
            print(transformed_pose.pose)
            return True, transformed_pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Get transformed pose failed to do the transform")
            return False, None

    def group_go_to_joint_states(self, group_name, goal, tolerance=0.01):
        """Set the joint states of a group as goal.

        :param group_name: str Controlled group name
        :param goal: list Joint states
        :param tolerance: float
        :return: bool
        """
        group = self._get_group_by_name(group_name)
        group.clear_pose_targets()
        group.go(list(goal), wait=True)
        group.stop()
        return self._wait_js_goal_execution(group_name, goal, tolerance)

    def _wait_js_goal_execution(self, group_name, js_goal, tol):
        js_temp = self.get_joint_states_of_group(group_name)
        cnt = 20
        while not rospy.is_shutdown() and cnt:
            rospy.sleep(0.1)
            js_curr = self.get_joint_states_of_group(group_name)
            if common.all_close(js_goal, js_curr, tol):
                return True
            if common.all_close(js_curr, js_temp, 0.001):
                cnt -= 1
            else:
                js_temp = js_curr
        return False

    @staticmethod
    def _group_go_to_predefined_target(group):
        try:
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            return True
        except moveit_commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

    def group_go_to_named_states(self, group_name, state_name):
        group = self._get_group_by_name(group_name)
        try:
            group.set_named_target(state_name)
            return self._group_go_to_predefined_target(group)
        except moveit_commander.MoveItCommanderException:
            return False

    def all_go_to_joint_states(self, goals):
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            ok = self.move_groups[i].go(goal, wait=True)
            if not ok:
                return False
        for group in self.move_groups:
            group.stop()
        return True

    def _execute_group_pose(self, group_name, goal, tolerance=0.01, constraint=''):
        """Make the planning group's eef move to given goal pose.

        :param group_name: str Planning group name
        :param goal: Pose Goal pose
        :param tolerance: float Tolerance for considering the goal has been reached
        :param constraint: str
        :return:
        """
        group = self._get_group_by_name(group_name)
        group.clear_pose_targets()
        group.set_pose_target(goal)
        try:
            constraints = self.get_group_orientation_constraints(group, constraint)
            group.set_path_constraints(constraints)
            group.go(wait=True)
        except self.commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            group.clear_path_constraints()
            return False

        group.stop()
        group.clear_pose_targets()
        group.clear_path_constraints()
        return self._wait_pose_goal_execution(group_name, goal, tolerance)

    def _relative_pose_to_absolute_pose(self, group_name, relative_pose, init_pose=None):
        """Convert a relative pose in local base frame to global base frame.

        :param group_name: str Planning group name
        :param relative_pose: Pose or List[float]
        :param init_pose: Initial pose of the robot before moving relatively
        :return: Pose
        """
        if not init_pose:
            current_pose = self.get_current_pose_of_group(group_name)
        else:
            current_pose = init_pose
        current_pose_mat = common.sd_pose(current_pose)  # current_pose_mat: T_be
        local_base_mat = transform.identity_matrix()
        local_base_mat[0:3, 3] = current_pose_mat[0:3, 3]  # local_base_mat: T_bl

        # Since T_be = T_bl * T_le, T_le = T_bl^-1 * T_be
        eef_local_mat = np.dot(np.linalg.inv(local_base_mat), current_pose_mat)  # T_le
        relative_pose_mat = common.sd_pose(relative_pose)  # T_ll'
        # T_be' = T_bl * T_ll' * T_l'e, note that T_l'e == T_le
        absolute_pose_mat = np.dot(np.dot(local_base_mat, relative_pose_mat), eef_local_mat)
        return common.to_ros_pose(absolute_pose_mat)

    def _eef_pose_to_absolute_pose(self, group_name, relative_pose, init_pose=None):
        """Convert a relative pose in eef frame to global base frame.

        :param group_name: str Planning group name
        :param relative_pose: Pose or List[float]
        :param init_pose: Initial pose of the robot before moving relatively
        :return: Pose
        """
        if not init_pose:
            current_pose = self.get_current_pose_of_group(group_name)
        else:
            current_pose = init_pose
        current_pose_mat = common.sd_pose(current_pose)
        relative_pose_mat = common.sd_pose(relative_pose)
        absolute_pose_mat = np.dot(current_pose_mat, relative_pose_mat)  # T_b1 * T_12 = T_b2
        return common.to_ros_pose(absolute_pose_mat)

    def group_goto_pose_goal_abs(self, group_name, goal, tolerance=0.01, constraint=''):
        """Move group to the goal pose wrt the global base frame

        :param group_name: Controlled group name
        :param goal: geometry_msgs.msg.Pose or PoseStamped
        :param tolerance:
        :param constraint: str, path constraint.
        :return: whether goal reached
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError('Goal of type {} is not defined'.format(type(goal)))

        return self._execute_group_pose(group_name, goal_pose, tolerance, constraint)

    def group_goto_pose_goal_rel(self, group_name, goal, tolerance=0.01, constraint=''):
        """Move group to the goal pose wrt the local base frame

        :param group_name: Controlled group name
        :param goal: geometry_msgs.msg.Pose or PoseStamped
        :param tolerance:
        :param constraint: str, path constraint.
        :return: whether goal reached
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError('Goal of type {} is not defined'.format(type(goal)))

        abs_goal = self._relative_pose_to_absolute_pose(group_name, goal_pose)
        return self._execute_group_pose(group_name, abs_goal, tolerance, constraint)

    def group_goto_pose_goal_eef(self, group_name, goal, tolerance=0.01, constraint=''):
        """Move group to the goal pose wrt the eef frame

        :param group_name: str Planning group name.
        :param goal: Pose/PoseStamped
        :param tolerance: float
        :param constraint: str, path constraint.
        :return:
        """
        if isinstance(goal, GeometryMsg.PoseStamped):
            goal_pose = goal.pose
        elif isinstance(goal, GeometryMsg.Pose):
            goal_pose = goal
        else:
            raise NotImplementedError

        abs_goal = self._eef_pose_to_absolute_pose(group_name, goal_pose)
        return self._execute_group_pose(group_name, abs_goal, tolerance, constraint)

    def _wait_pose_goal_execution(self, group_name, pose_goal, tolerance):
        group = self._get_group_by_name(group_name)
        pose_temp = common.regularize_pose(group.get_current_pose().pose)
        cnt = 20
        while not rospy.is_shutdown() and cnt:
            rospy.sleep(0.1)
            pose_curr = common.regularize_pose(group.get_current_pose().pose)
            # Here we only compare position, since quaternion value could be different for the same
            # rotation due to +/-
            position_goal = common.sd_position(pose_goal.position)
            position_curr = common.sd_position(pose_curr.position)
            position_temp = common.sd_position(pose_temp.position)
            if common.all_close(position_goal, position_curr, tolerance):
                return True
            if common.all_close(position_curr, position_temp, 0.001):
                cnt -= 1
            else:
                pose_temp = pose_curr
        return False

    def _execute_group_position(self, group_name, goal, tolerance=0.01):
        """Set the position of the tcp of a group as goal.

        :param group_name:
        :param goal:
        :param tolerance:
        :return:
        """
        group = self._get_group_by_name(group_name)
        group.clear_pose_targets()
        group.set_position_target(goal)
        try:
            group.go(wait=True)
        except self.commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

        group.stop()
        group.clear_pose_targets()

        current_position = self.get_current_position_of_group(group_name)
        # Standardize input before using all_close
        goal = common.sd_position(goal)
        current_position = common.sd_position(current_position)
        return common.all_close(goal, current_position, tolerance)

    def _to_absolute_position(self, group_name, relative_position):
        """Convert a relative position in eef frame to base frame.

        :param group_name: str Planning group name
        :param relative_position: Point or List[float]
        :return: ndarray
        """
        current_pose = self.get_current_pose_of_group(group_name)
        current_pose_mat = common.sd_pose(current_pose)
        relative_pose_mat = transform.identity_matrix()
        relative_pose_mat[0:3, 3] = common.sd_position(relative_position)
        absolute_pose_mat = np.dot(current_pose_mat, relative_pose_mat)  # T_b1 * T_12 = T_b2
        absolute_position = absolute_pose_mat[0:3, 3]
        return absolute_position

    def group_go_to_global_base_position(self, group_name, goal, tolerance=0.001):
        goal = common.sd_position(goal)
        return self._execute_group_position(group_name, goal, tolerance)

    def group_go_to_eef_position(self, group_name, goal, tolerance=0.001):
        goal = common.sd_position(goal)
        abs_goal = self._to_absolute_position(group_name, goal)
        return self._execute_group_position(group_name, abs_goal, tolerance)

    def group_shift(self, group_name, axis, goal):
        """Move the group along given axis and shift goal

        :param group_name:
        :param axis: Could be x y z r p y
        :param goal: float
        :return:
        """
        group = self._get_group_by_name(group_name)
        # 0,1,2,3,4,5 for x y z roll pitch yaw
        axis_list = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        axis_id = -1
        for i, a in enumerate(axis_list):
            if a == axis:
                axis_id = i
                break

        if axis_id < 0:
            raise NotImplementedError

        group.shift_pose_target(axis_id, goal)

        try:
            group.go(wait=True)
        except self.commander.MoveItCommanderException:
            group.stop()
            group.clear_pose_targets()
            return False

        group.stop()
        group.clear_pose_targets()
        return True

    def _all_go_to_pose_goal(self, goals):
        plans = []
        for i, goal in enumerate(goals):
            self.move_groups[i].set_pose_target(goal)
            plan = self.move_groups[i].plan()
            plans.append(plan)

        for plan, group in zip(plans, self.move_groups):
            group.execute(plan, wait=True)

        for group in self.move_groups:
            group.stop()
            group.clear_pose_targets()
        return True

    def _update_plan_time_stamps(self, group, plan, stamp):
        original_stamp = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        # points_num = len(plan.joint_trajectory.points)
        velocity_scale = original_stamp / stamp
        acceleration_scale = velocity_scale
        curr_state = self.commander.get_current_state()
        updated_plan = group.retime_trajectory(curr_state, plan, velocity_scale, acceleration_scale)
        # for i in range(points_num):
        #     plan.joint_trajectory.points[i].time_from_start = rospy.Duration.from_sec(t * (i + 1))
        return updated_plan

    def _build_cartesian_path_for_group(self, group_name, poses, stamp=None, allow_collisions=True):
        """Given waypoints in a list of geometry_msgs.Pose, plan a Cartesian path that
        goes through all waypoints.

        :param group_name: Group name for building a plan
        :param poses: geometry_msgs.PoseArray or List[Pose]
        :param stamp: Last time stamp from start
        :param allow_collisions: If true, allow collision along the path
        """
        group = self._get_group_by_name(group_name)
        if isinstance(poses, GeometryMsg.PoseArray):
            poses = poses.poses
        plan, fraction = group.compute_cartesian_path(poses, eef_step=0.01, jump_threshold=0,
                                                      avoid_collisions=not allow_collisions)
        # move_group_interface.h  L754
        if fraction < 0:
            rospy.logwarn('Path fraction less than 0.')
        if stamp:
            plan = self._update_plan_time_stamps(group, plan, stamp)
        return plan

    def _execute_group_plan(self, group_name, plan, wait=True):
        group = self._get_group_by_name(group_name)
        return group.execute(plan, wait)

    # def build_absolute_paths_for_all(self, all_poses, all_stamps=None, avoid_collisions=True):
    #     """
    #
    #     :param all_poses: List[List[Pose]] or List[PoseArray]
    #     :param all_stamps:
    #     :param avoid_collisions:
    #     :return:
    #     """
    #     all_plans = []
    #     if all_stamps:
    #         assert len(all_poses) == len(all_stamps), \
    #             "all_poses len {} is not equal to all_stamps len {}".format(len(all_poses), len(all_stamps))
    #     else:
    #         all_stamps = [None] * len(all_poses)
    #     for i, poses in enumerate(all_poses):
    #         if isinstance(poses, GeometryMsg.PoseArray):
    #             poses = poses.poses
    #         group_name = self.group_names[i]
    #         plan = self.build_absolute_path_for_group(group_name, poses, all_stamps[i], avoid_collisions)
    #         all_plans.append(plan)
    #     return all_plans
    #
    # def build_relative_paths_for_all(self, all_poses, all_stamps=None, avoid_collisions=True):
    #     all_abs_poses = []
    #     for i, rel_poses in enumerate(all_poses):
    #         if isinstance(rel_poses, GeometryMsg.PoseArray):
    #             rel_poses = rel_poses.poses
    #         group_name = self.group_names[i]
    #         init_pose = None
    #         abs_poses_of_group = []
    #         for rel_pose in rel_poses:
    #             abs_pose = self._eef_pose_to_absolute_pose(group_name, rel_pose, init_pose)
    #             abs_poses_of_group.append(abs_pose)
    #             init_pose = abs_pose
    #         all_abs_poses.append(abs_poses_of_group)
    #     return self.build_absolute_paths_for_all(all_abs_poses, all_stamps, avoid_collisions)

    # def execute_plans_for_all(self, plans):
    #     assert len(plans) == self.group_num
    #     for i, plan in enumerate(plans):
    #         group = self.move_groups[i]
    #         ok = group.execute(plan, wait=True)
    #         if not ok:
    #             return False
    #     return True

    def all_goto_pose_goals_abs(self, group_names, goals, tolerance, stamps=None, allow_collision=True):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            group_name = group_names[i]
            try:
                stamp = stamps[i] if stamps else None
            except IndexError:
                stamp = None
            plan = self._build_cartesian_path_for_group(group_name, [goal], stamp, allow_collision)
            if not self._execute_group_plan(group_name, plan):
                return False
        return True

    def all_goto_pose_goals_rel(self, group_names, goals, tolerance, stamps=None, allow_collision=True):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            group_name = group_names[i]
            try:
                stamp = stamps[i] if stamps else None
            except IndexError:
                stamp = None
            goal = self._relative_pose_to_absolute_pose(group_name, goal)
            plan = self._build_cartesian_path_for_group(group_name, [goal], stamp, allow_collision)
            if not self._execute_group_plan(group_name, plan, wait=True):
                return False
        return True

    def all_goto_pose_goals_eef(self, group_names, goals, tolerance, stamps=None, allow_collision=True):
        if isinstance(goals, GeometryMsg.PoseArray):
            goals = goals.poses
        assert len(goals) == self.group_num
        for i, goal in enumerate(goals):
            group_name = group_names[i]
            try:
                stamp = stamps[i] if stamps else None
            except IndexError:
                stamp = None
            goal = self._eef_pose_to_absolute_pose(group_name, goal)
            plan = self._build_cartesian_path_for_group(group_name, [goal], stamp, allow_collision)
            if not self._execute_group_plan(group_name, plan, wait=True):
                return False
        return True

    def add_box(self, group_name, box_name, box_pose, box_size, is_absolute, auto_suffix=False):
        group = self._get_group_by_name(group_name)
        box_pose_stamped = GeometryMsg.PoseStamped()
        box_pose_stamped.pose = box_pose
        if is_absolute:
            box_pose_stamped.header.frame_id = group.get_planning_frame()
        else:
            box_pose_stamped.header.frame_id = group.get_end_effector_link()
        if box_name == '':
            box_name = 'box'
        if auto_suffix:
            box_name += str(self._obj_suffix)
            self._obj_suffix += 1
        assert isinstance(box_size, GeometryMsg.Point)
        # size must be iterable
        self.scene.add_box(box_name, box_pose_stamped, size=(box_size.x, box_size.y, box_size.z))

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < 3) and not rospy.is_shutdown():
            if box_name in self.scene.get_known_object_names():
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def attach_box(self, group_name, eef_group_name, box_name, box_pose, box_size):
        ok = self.add_box(group_name, box_name, box_pose, box_size, is_absolute=True, auto_suffix=False)
        if not ok:
            return False
        group = self._get_group_by_name(group_name)
        grasping_group = eef_group_name
        touch_links = self.commander.get_link_names(group=grasping_group)
        self.scene.attach_box(group.get_end_effector_link(), box_name, touch_links=touch_links)
        return True

    def detach_object(self, group_name, obj_name):
        group = self._get_group_by_name(group_name)
        self.scene.remove_attached_object(group.get_end_effector_link(), name=obj_name)
        return True

    def remove_object(self, obj_name, is_exact=False):
        if is_exact:
            self.scene.remove_world_object(obj_name)
        else:
            for name in self.scene.get_known_object_names():
                if obj_name in name:
                    self.scene.remove_world_object(name)
        return True

    def add_plane(self, group_name, plane_name, plane_pose, plane_normal, auto_suffix=False):
        group = self._get_group_by_name(group_name)
        if plane_name == '':
            plane_name = 'plane'
        if auto_suffix:
            plane_name += str(self._obj_suffix)
            self._obj_suffix += 1
        plane_pose_stamped = GeometryMsg.PoseStamped()
        plane_pose_stamped.pose = plane_pose
        plane_pose_stamped.header.frame_id = group.get_planning_frame()
        self.scene.add_plane(plane_name, plane_pose_stamped, normal=(plane_normal.x, plane_normal.y, plane_normal.z))
        return True

    @staticmethod
    def get_group_orientation_constraints(group, constraint=''):
        """Generate an orientation constraint object for a given planning group based on
        the constraint descriptors.

        :param group: The planning group
        :param constraint: str A string containing 'r', 'p', or 'y', or their combination
        :return: moveit_msgs.Constraints
        """
        ref_pose = group.get_current_pose().pose
        constraints = MoveItMsg.Constraints()

        oc = MoveItMsg.OrientationConstraint()
        oc.orientation.x = ref_pose.orientation.x
        oc.orientation.y = ref_pose.orientation.y
        oc.orientation.z = ref_pose.orientation.z
        oc.orientation.w = ref_pose.orientation.w

        no_constraint = math.pi * 4
        if 'r' in constraint:
            oc.absolute_x_axis_tolerance = 0.2  # roll
        else:
            oc.absolute_x_axis_tolerance = no_constraint

        if 'p' in constraint:
            oc.absolute_y_axis_tolerance = 0.2  # pitch
        else:
            oc.absolute_y_axis_tolerance = no_constraint

        if 'y' in constraint:
            oc.absolute_z_axis_tolerance = 0.2  # yaw
        else:
            oc.absolute_z_axis_tolerance = no_constraint

        oc.weight = 1.0
        oc.link_name = group.get_end_effector_link()
        oc.header.frame_id = group.get_end_effector_link()
        oc.header.stamp = rospy.Time.now()
        constraints.orientation_constraints.append(oc)

        return constraints
