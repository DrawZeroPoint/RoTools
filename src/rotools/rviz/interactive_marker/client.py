#!/usr/bin/env python
from __future__ import print_function

import rospy

import rotools.rviz.interactive_marker.interface as interface

from roport.srv import *


class InteractiveMarkerClient(object):
    def __init__(self, marker_info):
        """Initialize the InteractiveMarkerClient.

        Args:
            marker_info: list[dict]
        """
        super(InteractiveMarkerClient, self).__init__()

        self._interfaces = []
        for info in marker_info:
            marker_interface = interface.InteractiveMarkerInterface(info)
            marker_interface.start()
            self._interfaces.append(marker_interface)

        self._execute_marker_reset_srv = rospy.Service(
            "execute_interactive_marker_reset",
            ExecuteBinaryAction,
            self.execute_marker_reset_handle,
        )

    def execute_marker_reset_handle(self, req):
        resp = ExecuteBinaryActionResponse()
        # req = ExecuteBinaryActionRequest()
        for i in self._interfaces:
            if not i.reset_marker_pose():
                resp.result_status = resp.FAILED
                return resp
        resp.result_status = resp.SUCCEEDED
        return resp
