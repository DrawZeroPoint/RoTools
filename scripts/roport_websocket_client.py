#!/usr/bin/env python

import rospy

from rotools.websocket.core.client import WebsocketClient

from rotools.utility.common import (
    get_param,
    pretty_print_configs,
    is_ip_valid,
    is_port_valid,
)


if __name__ == "__main__":
    try:
        rospy.init_node("roport_websocket_client", anonymous=True)

        configs = {
            "ip": get_param("~ip"),
            "port": get_param("~port"),
            "from_client_topics": get_param("~from_client_topics"),
            "to_client_topics": get_param("~to_client_topics"),
        }

        if not is_ip_valid(configs["ip"]) or not is_port_valid(configs["port"]):
            exit(-1)

        pretty_print_configs(configs)
        client = WebsocketClient(configs)
        rospy.loginfo("RoPort Websocket Client ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
