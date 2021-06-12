#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.sensing.core.server import SensingServer
from rotools.utility.common import get_param


if __name__ == "__main__":
    try:
        rospy.init_node('roport_sensing_server')
        # You only need to modify the config to apply this to new robots
        device_names = get_param('~device_names')
        assert device_names is not None, print("No sensing device available")
        algorithm_names = get_param('~algorithm_names')
        algorithm_ports = get_param('~algorithm_ports')
        assert algorithm_ports is not None, print("No sensing algorithm available")
        if algorithm_names and algorithm_ports:
            assert len(algorithm_names) == len(algorithm_ports), print('Algorithm names mismatch ports')
        config = {
            'device_names': device_names,
            'algorithm_names': algorithm_names,
            'algorithm_ports': algorithm_ports,
        }
        rospy.loginfo("RoPort: Sensing server configurations: \n{}".format(config))
        server = SensingServer(config)
        rospy.loginfo("RoPort: Sensing server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
