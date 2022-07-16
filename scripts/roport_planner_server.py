#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.planner.core.server import PlannerServer
from rotools.utility.common import get_param, pretty_print_configs


if __name__ == "__main__":
    try:
        rospy.init_node("roport_planner_server")
        configs = {
            "mode": get_param("~mode"),
            "control_topics": get_param("~control_topics"),
            "rate": get_param("~rate"),
            "algorithm_port": get_param("~algorithm_port"),
            "robot_initial_poses": get_param("~robot_initial_poses"),
            "planning_initial_poses": get_param("~planning_initial_poses"),
        }
        pretty_print_configs(configs)

        server = PlannerServer(configs)
        rospy.loginfo("RoPort: Planner server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
