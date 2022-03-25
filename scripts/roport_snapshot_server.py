#!/usr/bin/env python
import rospy

from rotools.snapshot.core.server import SnapshotServer
from rotools.utility.common import get_param, pretty_print_configs


if __name__ == "__main__":
    try:
        rospy.init_node('roport_snapshot_server')

        configs = {
            'js_topics': get_param('~js_topics'),
            'odom_topics': get_param('~odom_topics'),
            'rgb_topics': get_param('~rgb_topics'),
            'depth_topics': get_param('~depth_topics'),
            'save_dir': get_param('~save_dir'),
        }

        pretty_print_configs(configs)
        server = SnapshotServer(configs)
        rospy.loginfo("RoPort Snapshot Server ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
