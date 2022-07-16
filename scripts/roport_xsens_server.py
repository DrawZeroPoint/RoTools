#!/usr/bin/env python
from __future__ import print_function

import rospy

from rotools.xsens.core.server import XsensServer
from rotools.utility.common import (
    get_param,
    pretty_print_configs,
    is_ip_valid,
    is_port_valid,
)


if __name__ == "__main__":
    try:
        rospy.init_node("roport_xsens_server")
        configs = {
            "udp_ip": get_param("~ip", ""),
            "udp_port": get_param("~port", 9763),
            "ref_frame": get_param("~ref_frame", "world"),
            "scaling": get_param("~scaling", 1.0),
            "rate": get_param("~rate", 60.0),
            "prop": get_param("~prop", False),
        }

        # If udp_ip is not given, get local IP as UDP IP
        if configs["udp_ip"] == "":
            import socket

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            configs["udp_ip"] = s.getsockname()[0]
            s.close()

        if not is_ip_valid(configs["udp_ip"]) or not is_port_valid(configs["port"]):
            exit(-1)

        pretty_print_configs(configs)
        server = XsensServer(configs)
        rospy.loginfo("RoPort: Xsens server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
