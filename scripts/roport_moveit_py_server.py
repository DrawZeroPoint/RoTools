#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.moveit.core.server import MoveItServer
from rotools.utility.common import get_param, pretty_print_configs


if __name__ == "__main__":
    try:
        rospy.init_node("roport_moveit_py_server")
        # You only need to modify the config to apply this to new robots
        group_names = get_param("~group_names")
        assert group_names is not None
        ee_links = get_param("~ee_links")
        # Will be the base frame of the planning group if not given
        ref_frames = get_param("~ref_frames")
        configs = {
            "robot_description": "robot_description",
            "ns": "",
            "group_names": group_names,
            "ee_links": ee_links,
            "ref_frames": ref_frames,
        }
        pretty_print_configs(configs)
        server = MoveItServer(configs)
        rospy.loginfo("RoPort: MoveIt python server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
