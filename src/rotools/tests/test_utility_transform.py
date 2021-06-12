#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import tf.transformations
import math
import numpy as np

import rotools.utility.transform as transform


class Test(unittest.TestCase):

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
        R = transform.rotation_matrix(-180 * np.pi / 180, (1, 0, 0))
        # print(np.array2string(R, separator=', '))
        x_20 = transform.quaternion_from_matrix(R)
        print(x_20)

        R = transform.euler_matrix(180 * np.pi / 180., 0, 90 * np.pi / 180.)
        x_20 = transform.quaternion_from_matrix(R)
        print(x_20)

        sin_20 = math.sin(20. / 180 * np.pi)  # 20. makes sure the result is a float
        cos_20 = math.cos(20. / 180 * np.pi)

        homogeneous_matrix = np.array([
            [0., 0., 1., 0.084415],
            [1., 0., 0., 0.],
            [0., 1., 0, 0.098503+0.093313],
            [0., 0., 0., 1.]
        ], dtype=float)
        q = transform.quaternion_from_matrix(homogeneous_matrix)
        # print(q)

        # walker base link to head l3
        homogeneous_matrix = np.array([
            [0., 0., 1., 0.084415],
            [1., 0., 0., 0.],
            [0., 1., 0, 0.098503+0.093313],
            [0., 0., 0., 1.]
        ], dtype=float)
        q = transform.quaternion_from_matrix(homogeneous_matrix)
        print("base_link to l3\n", q)

        """
        Commonly used rotation quaternions along z axis
        45 deg: [0.         0.         0.38268343 0.92387953]
        90 deg: [0.         0.         0.70710678 0.70710678]
        180 deg: [0.000000e+00 0.000000e+00 1.000000e+00 6.123234e-17]
        270 deg: [ 0.          0.         -0.70710678  0.70710678]
        """
        R1 = transform.rotation_matrix(45 * np.pi / 180., (0, 0, 1))
        q1 = transform.quaternion_from_matrix(R1)
        print('q1\n', q1)

        Q1 = transform.quaternion_matrix([0.0, 0.0, 0.0533170029521, 0.998577654362])
        print(transform.euler_from_matrix(Q1))

        Q2 = transform.quaternion_matrix([0.0, 0.0, 0.0573195181787, 0.998355925083])
        print(transform.euler_from_matrix(Q2))

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


if __name__ == '__main__':
    unittest.main()
