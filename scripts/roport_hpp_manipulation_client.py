#!/usr/bin/env python

import sys
import rospy
import subprocess

from rotools.hpp.manipulation.client import HPPManipulationClient

from rotools.utility.common import get_param, pretty_print_configs


if __name__ == '__main__':
    server_process = None
    if sys.version_info >= (3, 2):
        server_process = subprocess.Popen(["hppcorbaserver"], start_new_session=True)
    else:
        rospy.logwarn("You need to manually run 'hppcorbaserver' before launching this program")

    try:
        rospy.init_node('roport_hpp_manipulation_client')

        configs = {
            'env_name': get_param('~env_name'),
            'env_pkg_name': get_param('~env_pkg_name'),
            'env_surface': get_param('~env_surface'),
            'object_name': get_param('~object_name'),
            'object_pkg_name': get_param('~object_pkg_name'),
            'object_surface': get_param('~object_surface'),
            'object_handle': get_param('~object_handle'),
            'robot_name': get_param('~robot_name'),
            'robot_pkg_name': get_param('~robot_pkg_name'),
            'robot_urdf_name': get_param('~robot_urdf_name'),
            'robot_srdf_name': get_param('~robot_srdf_name'),
            'robot_bound': get_param('~robot_bound'),
            'object_bound': get_param('~object_bound'),
            'gripper_name': get_param('~gripper_name'),
            'fingers': get_param('~fingers'),
            'finger_joints': get_param('~finger_joints'),
            'finger_joint_values': get_param('~finger_joint_values'),
            'joint_state_topic': get_param('~joint_state_topic'),
            'odom_topic': get_param('~odom_topic'),
            'object_pose_topic': get_param('~object_pose_topic'),
            'joint_cmd_topic': get_param('~joint_cmd_topic'),
            'base_cmd_topic': get_param('~base_cmd_topic'),
            'enable_viewer': get_param('~enable_viewer'),
            'reduction_ratio': get_param('~reduction_ratio', 0.2),
        }

        viewer_process = None
        if configs['enable_viewer']:
            if sys.version_info >= (3, 2):
                viewer_process = subprocess.Popen(["gepetto-gui", "-c", "basic"],
                                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                                  start_new_session=True)
            else:
                rospy.logwarn("You need to manually run 'gepetto-gui -c basic' before launching this program")

        pretty_print_configs(configs)
        client = HPPManipulationClient(configs)
        rospy.loginfo("RoPort HPP Manipulation Client ready.")
        rospy.spin()
        if server_process is not None:
            server_process.kill()
        if viewer_process is not None:
            viewer_process.kill()
    except rospy.ROSInterruptException as e:
        print(e)
