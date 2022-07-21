#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest
import math
import numpy as np

import geometry_msgs.msg as geo_msg

import rotools.utility.robotics as robotics


class Test(unittest.TestCase):
    def test_adjoint_T(self):
        T = np.array(
            [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 3], [0, 0, 0, 1]], dtype=float
        )
        Ad_T = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 0, -1, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 3, 1, 0, 0],
                [3, 0, 0, 0, 0, -1],
                [0, 0, 0, 0, 1, 0],
            ],
            dtype=float,
        )
        self.assertTrue(np.allclose(robotics.Adjoint(T), Ad_T))

    def test_adjoint_V(self):
        V = np.array([1, 2, 3, 4, 5, 6])
        ad_V = np.array(
            [
                [0, -3, 2, 0, 0, 0],
                [3, 0, -1, 0, 0, 0],
                [-2, 1, 0, 0, 0, 0],
                [0, -6, 5, 0, -3, 2],
                [6, 0, -4, 3, 0, -1],
                [-5, 4, 0, -2, 1, 0],
            ]
        )
        self.assertTrue(np.allclose(robotics.ad(V), ad_V))

    def test_axis_angle3(self):
        exp_coord3 = np.array([1, 2, 3])
        result = np.array([0.26726124, 0.53452248, 0.80178373]), 3.7416573867739413
        self.assertTrue(np.allclose(robotics.axis_angle3(exp_coord3)[0], result[0]))
        self.assertTrue(np.allclose(robotics.axis_angle3(exp_coord3)[1], result[1]))

    def test_axis_angle6(self):
        exp_c6 = np.array([1, 0, 0, 1, 2, 3])
        result = np.array([1.0, 0.0, 0.0, 1.0, 2.0, 3.0]), 1.0
        self.assertTrue(np.allclose(robotics.axis_angle6(exp_c6)[0], result[0]))
        self.assertTrue(np.allclose(robotics.axis_angle6(exp_c6)[1], result[1]))


if __name__ == "__main__":
    unittest.main()
