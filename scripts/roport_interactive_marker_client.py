#!/usr/bin/env python

import rospy

from rotools.rviz.interactive_marker.client import InteractiveMarkerClient

from rotools.utility.common import (
    get_param,
    pretty_print_configs,
)

if __name__ == "__main__":
    try:
        rospy.init_node("roport_interactive_marker_client", anonymous=True)

        configs = {
            "marker_names": get_param("marker_names"),
            "target_frame_ids": get_param("target_frame_ids"),
            "reference_frame_ids": get_param("reference_frame_ids"),
            "scales": get_param("scales"),
            "publish_topics": get_param("publish_topics"),
        }

        pretty_print_configs(configs)

        marker_info = []
        try:
            for i in range(len(configs["marker_names"])):
                info = {
                    "name": configs["marker_names"][i],
                    "target_frame_id": configs["target_frame_ids"][i],
                    "reference_frame_id": configs["reference_frame_ids"][i],
                    "scale": configs["scales"][i],
                    "publish_topic": configs["publish_topics"][i],
                }
                marker_info.append(info)
        except IndexError as e:
            rospy.logerr(e)

        client = InteractiveMarkerClient(marker_info)
        rospy.loginfo("RoPort Interactive Marker Client ready.")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
