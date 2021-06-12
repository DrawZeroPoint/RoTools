#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import numpy as np

import rotools.simulation.webots as webots_interface
import rotools.utility.transform as transform


class Test(unittest.TestCase):

    def test_ur_interface(self):
        """Before carrying this test, make sure the controller of the robot is
        chosen as <extern>, and the Webots simulation scene is running with a
        UR arm in scene.

        """
        np.set_printoptions(precision=4)
        ur_interface = webots_interface.WebotsInterfaceUR()
        js = ur_interface.get_joint_state()
        print('ur init joint state \n', js)

        pi_2 = np.pi / 2
        q = [0, -0.8, 0.8, 0, pi_2, 0]
        ur_interface.set_joint_max_velocity(1)
        ur_interface.go_to_joint_state(q)

        js = ur_interface.get_joint_state()
        print('ur current joint state \n', js)

        # Transformation matrix from base frame to TCP frame
        T = ur_interface.get_tcp_pose()
        print('ur tcp pose \n', T)
        t = T[:3, 3]
        q = transform.quaternion_from_matrix(T)
        print('ur tcp in translation & quaternion \n', t, q)


if __name__ == '__main__':
    unittest.main()
