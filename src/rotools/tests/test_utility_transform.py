#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import tf.transformations
import math
import numpy as np

import rotools.utility.transform as transform
import rotools.utility.common as common


class Test(unittest.TestCase):
    """
    DO NOT USE PRINT TO SHOW VALUES, THEY COULD COME IN UNEXPECTED ORDER, USE ASSET.
    """

    def test_transform(self):
        """
        5 * np.pi / 180, (0, 1, 0)  [0., 0.04361939, 0., 0.99904822]
        10 * np.pi / 180, (0, 1, 0)  [0., 0.08715574,  0., 0.9961947 ]
        15 * np.pi / 180, (0, 1, 0)  [0.         0.13052619 0.         0.99144486 ]
        20 * np.pi / 180, (0, 1, 0)  [0.         0.17364818 0.         0.98480775]
        40 * np.pi / 180, (0, 1, 0)  [0.         0.34202014 0.         0.93969262]
        90 * np.pi / 180, (0, 1, 0)  [0.         0.70710678 0.         0.70710678]
        -90 * np.pi / 180, (0, 1, 0)  [0.         -0.70710678 0.         0.70710678]
        """
        homogeneous_matrix = np.array([
            [0., 0., 1., 0.084415],
            [1., 0., 0., 0.],
            [0., 1., 0, 0.098503 + 0.093313],
            [0., 0., 0., 1.]
        ], dtype=float)
        q = transform.quaternion_from_matrix(homogeneous_matrix)

        # walker base link to head l3
        homogeneous_matrix = np.array([
            [0., 0., 1., 0.084415],
            [1., 0., 0., 0.],
            [0., 1., 0, 0.098503 + 0.093313],
            [0., 0., 0., 1.]
        ], dtype=float)
        q = transform.quaternion_from_matrix(homogeneous_matrix)
        print("base_link to l3\n", q)

        """
        Commonly used rotation quaternions along z axis
        45 deg: [0.         0.         0.38268343 0.92387953]
        90 deg: [0.         0.         0.70710678 0.70710678]
        135 deg: [0.         0.         0.92387953 0.38268343]
        180 deg: [0.000000e+00 0.000000e+00 1.000000e+00 6.123234e-17]
        270 deg: [ 0.          0.         -0.70710678  0.70710678]
        """
        # Franka robot install on non-default pose (curiosity's left arm)
        qm_left = transform.quaternion_matrix([-0.40318, 0.52543, 0.097796, 0.74283])  # T_base_to_install
        print('qm_left\n', qm_left)
        g_vector = np.array([0., 0., -9.81, 0])  # G_base
        g_vector_t = np.dot(qm_left.transpose(), g_vector)  # G_install = T_install_to_base * G_base
        print(qm_left.transpose())
        print('left arm g_vector_t\n', g_vector_t)

        # Franka robot install on non-default pose (curiosity's right arm)
        qm_right = transform.quaternion_matrix([0.40318, 0.52543, -0.097796, 0.74283])  # T_base_to_install
        g_vector = np.array([0., 0., -9.81, 0])  # G_base
        g_vector_t = np.dot(qm_right.transpose(), g_vector)  # G_install = T_install_to_base * G_base
        print('right arm g_vector_t\n', g_vector_t)

        print(np.linalg.norm(np.subtract(g_vector_t, g_vector)))

        # Calculate the orientation of the CURI arms regarding the base frame
        qm_left_new = transform.quaternion_matrix(
            [-0.5824349629454723, 0.3718840519259553, 0.2206752352096998, 0.688312549534299])
        print(qm_left_new)
        print(np.rad2deg(transform.euler_from_matrix(qm_left_new)))

        # FR Wheel joint axis
        rotated_axis = np.dot(transform.euler_matrix(0, np.deg2rad(-30), np.deg2rad(-45), 'rxyz'),
                              np.array([0., 1., 0., 0.]))
        print(rotated_axis)

    def test_quaternion_multiply(self):
        """This example shows the usage of quaternion_multiply.

        Given transforms Tb1 and Tbs, Ts1 is calculated by Ts1 = T1s.T = (Tb1.t * Tbs).T
        Notice that here '*' must be implemented with np.dot, and for rotation matrix,
        T.T == T^-1
        """
        Tb1 = transform.rotation_matrix(45 * np.pi / 180., (0, 0, 1))
        Tbs = transform.rotation_matrix(90 * np.pi / 180., (0, 1, 0))
        T1s = np.dot(np.transpose(Tb1), Tbs)
        Ts1 = np.transpose(T1s)

        q_b1 = transform.quaternion_from_matrix(Tb1)
        q_bs = transform.quaternion_from_matrix(Tbs)
        q_s1 = transform.quaternion_from_matrix(Ts1)
        q_m = tf.transformations.quaternion_multiply(q_bs, q_s1)
        for i in range(4):
            self.assertAlmostEqual(q_b1[i], q_m[i])

    def test_2d_rotation(self):
        R = transform.rotation_matrix(-20 * np.pi / 180, (0, 0, 1))
        v0_t = np.dot(R, np.array([1, 2, 0, 1]).T)
        v1_t = np.dot(R, np.array([1, -2, 0, 1]).T)
        v2_t = np.dot(R, np.array([-1, -2, 0, 1]).T)
        v3_t = np.dot(R, np.array([-1, 2, 0, 1]).T)
        # print(v0_t, v1_t, v2_t, v3_t)

    def test_most_used_transforms(self):
        """This test output well-used transformations around x-, y-, and z- axes in quaternion"""
        # Only rotate around +Y axis
        q = transform.quaternion_from_matrix(transform.euler_matrix(0, -np.pi * 0.5, 0))  # -90
        self.assertTrue(common.all_close(q, [0., -0.70710678, 0., 0.70710678]))
        # Only rotate around +X axis
        q = transform.quaternion_from_matrix(transform.euler_matrix(np.pi, 0, 0))  # 180
        self.assertTrue(common.all_close(q, [1., 0., 0., 0.]))


if __name__ == '__main__':
    unittest.main()
