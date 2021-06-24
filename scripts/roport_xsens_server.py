#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.xsens.core.server import XsensServer
from rotools.utility.common import get_param


if __name__ == "__main__":
    try:
        rospy.init_node('roport_xsens_server')
        udp_ip = get_param('~udp_ip')
        udp_port = get_param('~udp_port')
        ref_frame = get_param('~ref_frame')
        scaling_factor = get_param('~scaling_factor')

        config = {
            'udp_ip': udp_ip,
            'udp_port': udp_port,
            'ref_frame': ref_frame,
            'scaling_factor': scaling_factor,
        }
        rospy.loginfo("RoPort: Configs: \n{}".format(config))
        rospy.loginfo("RoPort: Xsens server ready to receive streaming.")
        server = XsensServer(config)
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
