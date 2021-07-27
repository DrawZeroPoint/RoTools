#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.xsens.core.server import XsensServer
from rotools.utility.common import get_param, pretty_print_configs


if __name__ == "__main__":
    try:
        rospy.init_node('roport_xsens_server')
        configs = {
            'udp_ip': get_param('~udp_ip'),
            'udp_port': get_param('~udp_port'),
            'ref_frame': get_param('~ref_frame'),
            'scaling': get_param('~scaling'),
            'rate': get_param('~rate'),
            'detail': get_param('~detail', False),
            'prop': get_param('~prop', False)
        }
        pretty_print_configs(configs)
        server = XsensServer(configs)
        rospy.loginfo("RoPort: Xsens server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
