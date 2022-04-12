#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.simulation.mujoco import MuJoCoServer
from rotools.utility.common import get_param, get_path, pretty_print_configs

if __name__ == "__main__":
    try:
        rospy.init_node('roport_mujoco_server')

        model_path = '.mujoco/mujoco214/model/humanoid/humanoid.xml'

        configs = {
            'model_path': get_path('model_path', model_path),
            'kinematics_path': '',
            'actuator_path': '',
            'enable_viewer': get_param('enable_viewer', True),
            'publish_rate': get_param('publish_rate', 60.),
        }

        pretty_print_configs(configs)
        server = MuJoCoServer(configs)
        rospy.loginfo("Roport MuJoCo server ready (only for demonstration).")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
