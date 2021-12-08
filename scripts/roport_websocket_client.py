#!/usr/bin/env python

import rospy

from rotools.web.core.client import WebsocketROSClient

from rotools.utility.common import get_param, pretty_print_configs


if __name__ == '__main__':
    try:
        rospy.init_node('websocket_client', anonymous=True)

        configs = {
            'ip': get_param('~ip'),
            'port': get_param('~port'),
            'downstream_list': get_param('~downstream_list'),
            'upstream_list': get_param('~upstream_list'),
        }

        if configs['ip'] is None or configs['port'] is None:
            rospy.logerr("IP or port is not set properly")
            exit(-1)

        pretty_print_configs(configs)
        client = WebsocketROSClient(configs)
        rospy.loginfo("RoPort: Websocket client ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
