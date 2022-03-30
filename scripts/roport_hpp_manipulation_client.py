#!/usr/bin/env python

import rospy

from rotools.hpp.manipulation.client import HPPManipulationClient

from rotools.utility.common import get_param, pretty_print_configs


if __name__ == '__main__':
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
            'joint_cmd_topic': get_param('~joint_cmd_topic'),
            'base_cmd_topic': get_param('~base_cmd_topic'),
        }

        pretty_print_configs(configs)
        client = HPPManipulationClient(configs)
        rospy.loginfo("RoPort HPP Manipulation Client ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
