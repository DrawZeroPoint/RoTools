#!/usr/bin/env python
from __future__ import print_function

import rospy

from roport.srv import *

import rotools.sensing.core.interface as interface


class SensingServer(object):

    def __init__(self, kwargs):

        super(SensingServer, self).__init__()

        self.interface = interface.SensingInterface(**kwargs)

        # Sensing services
        self._srv_sense_pose = rospy.Service('sense_manipulation_poses', SenseManipulationPoses, self.sense_pose_handle)

        # Information getters

        # General utilities
        self._srv_visualize_pose = rospy.Service('visualize_pose', VisualizePose, self.visualize_pose_handle)

    def sense_pose_handle(self, req):
        resp = SenseManipulationPosesResponse()
        try:
            ok, poses, best_pose = self.interface.sense_manipulation_poses(req.device_names, req.algorithm_id)
            if ok:
                resp.result_status = resp.SUCCEEDED
                resp.poses = poses
                resp.best_pose = best_pose
            else:
                resp.result_status = resp.FAILED
        except IndexError:
            resp.result_status = resp.FAILED
        return resp

    def visualize_pose_handle(self, req):
        resp = VisualizePoseResponse()
        self.interface.visualize_pose(req.pose, req.frame)
        resp.result_status = resp.SUCCEEDED
        return resp
