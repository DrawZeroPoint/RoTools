#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import math
import numpy as np

import geometry_msgs.msg as GeometryMsg

import rotools.utility.common as common


class Test(unittest.TestCase):

    def test_sd_pose(self):
        """
        Initial pose of franka panda_arm group
        position:
          x: 0.307019570052
          y: -5.22132961561e-12
          z: 0.590269558277
        orientation:
          x: 0.923955699469
          y: -0.382499497279
          z: 1.3249325839e-12
          w: 3.20041176635e-12
        :return:
        """
        ros_pose = GeometryMsg.Pose()
        ros_pose.position.x = 0.307019570052
        ros_pose.position.y = 0
        ros_pose.position.z = 0.590269558277
        ros_pose.orientation.x = 0.923955699469
        ros_pose.orientation.y = -0.382499497279
        ros_pose.orientation.z = 0
        ros_pose.orientation.w = 0

        pose_mat = common.sd_pose(ros_pose)
        re_pose = common.to_ros_pose(pose_mat)
        self.assertTrue(common.all_close(ros_pose, re_pose, 0.001))

    def test_relative_transform(self):
        """
        :return:
        """
        pose_1 = common.sd_pose(np.array([0.307, 0, 0.59, 0.924, -0.382, 0, 0]))
        pose_2 = common.sd_pose(np.array([0.307, 0, 0.59, -0.708, 0.706, 0, 0]))
        rel_trans = common.sd_pose(np.array([0, 0, 0, 0., 0., 0.383, 0.924]))
        trans = np.dot(pose_1, rel_trans)
        ros_pose_2 = common.to_ros_pose(pose_2)
        print(trans, '\n', pose_2, '\n', ros_pose_2)
        self.assertTrue(common.all_close(common.to_ros_pose(trans), ros_pose_2, 0.01))


if __name__ == '__main__':
    unittest.main()
