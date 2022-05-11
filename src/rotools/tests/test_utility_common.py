#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import math
import numpy as np

import geometry_msgs.msg as geo_msg
import rospy
import sensor_msgs.msg as sen_msg

import rotools.utility.common as common


class Test(unittest.TestCase):

    def test_print(self):
        common.print_debug('This is a debug msg')
        common.print_info('This is a info msg')
        common.print_warn('This is a warn msg')
        common.print_warning('This is a warning msg')
        common.print_error('This is a error msg')

    def test_all_close(self):
        values_a = [1, 2, 4]
        values_b = np.array([1, 2, 3])
        self.assertFalse(common.all_close(values_a, values_b))

        values_a = geo_msg.Pose()
        values_b = geo_msg.PoseStamped()
        self.assertTrue(common.all_close(values_a, values_b))
        with self.assertRaises(ValueError):
            common.all_close(values_a, values_b, 0)

        values_a = [1, 2]
        values_b = np.array([1, 2, 3])
        with self.assertRaises(ValueError):
            self.assertFalse(common.all_close(values_a, values_b))

    def test_to_list(self):
        p = geo_msg.Pose()
        self.assertEqual([0, 0, 0, 0, 0, 0, 0], common.to_list(p))

    def test_offset_ros_pose(self):
        pose = geo_msg.Pose()
        offset = [1, -1, 2]
        new_pose = common.offset_ros_pose(pose, offset)
        self.assertEqual([1, -1, 2, 0, 0, 0, 0], common.to_list(new_pose))
        pose_stamped = geo_msg.PoseStamped()
        new_pose_stamped = common.offset_ros_pose(pose_stamped, offset)
        self.assertEqual([1, -1, 2, 0, 0, 0, 0], common.to_list(new_pose_stamped))

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
        ros_pose = geo_msg.Pose()
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

    # def test_create_publishers(self):
    #     rospy.init_node('test_create_publishers', anonymous=True)
    #     id_list = ['a', 'b', 'c', 'd', 'e', 'f']
    #     topic_types = [sen_msg.JointState] * 6
    #     publishers = common.create_publishers("", id_list, topic_types)
    #     while True:
    #         for i, publisher in enumerate(publishers.values()):
    #             msg = sen_msg.JointState()
    #             msg.header.stamp = rospy.Time.now()
    #             msg.name.append(id_list[i])
    #             msg.position.append(i)
    #             msg.velocity.append(i)
    #             msg.effort.append(i)
    #             publisher.publish(msg)
    #         rospy.sleep(rospy.Duration.from_sec(0.001))

    def test_to_ros_orientation(self):
        with self.assertRaises(ValueError):
            common.to_ros_orientation([0, 0, 0, 0], check=True)


if __name__ == '__main__':
    unittest.main()
