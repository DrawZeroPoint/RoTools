#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
A fast tool to calculate M in PoE model
"""

from __future__ import print_function

import unittest

import numpy as np

import rotools.utility.transform as transform


class Test(unittest.TestCase):

    def test_create_walker_arm_model(self):
        """Create a product of exponential model from urdf file.
        The following data is from walker.urdf, the left arm

        """
        # [x, y, z, r, p, y] from urdf
        # we let the static base frame {s} of the arm coincide with joint 1,
        # hence the transformation from base link to {s} is fixed and equal to base_to_l1
        base_to_l1 = np.array([0, 0.24142, 0.018386, -1.931E-07, 1.3073, 1.5708])
        base_to_l1_matrix = transform.euler_matrix(base_to_l1[3], base_to_l1[4], base_to_l1[5])
        base_to_l1_matrix[:3, 3] = base_to_l1[:3]
        print('matrix from base to static\n', np.array2string(base_to_l1_matrix, separator=', '))

        s_to_l1 = np.array([0, 0, 0, 0, 0, 0])
        l1_to_l2 = np.array([0, 0, 0, -1.5708, 1.5708, 0])
        l2_to_l3 = np.array([-0.00064368, -0.2285, 0, 1.5708, 6.1653E-11, -2.7756E-17])
        l3_to_l4 = np.array([0.025422, 0, 0, -1.5708, -0.046339, 3.1416])
        l4_to_l5 = np.array([0.024818, -0.22241, 0, -1.5708, -1.5708, -3.1416])
        l5_to_l6 = np.array([0, 0, 0, 1.5708, -1.5708, 0])
        l6_to_l7 = np.array([0, 0, 0, 1.5708, 3.0389E-17, 6.4236E-24])

        trans = [s_to_l1, l1_to_l2, l2_to_l3, l3_to_l4, l4_to_l5, l5_to_l6, l6_to_l7]
        M = transform.identity_matrix()  # home matrix
        for t in trans:
            r_matrix = transform.euler_matrix(t[3], t[4], t[5])  # 4x4
            t_vector = t[:3]
            r_matrix[:3, 3] = t_vector
            # note that here we use r_matrix right product l1_to_l7
            M = np.dot(M, r_matrix)
        print('The M matrix for walker`s left arm is\n', np.array2string(M, separator=', '))

        trans = [l1_to_l2, l2_to_l3, l3_to_l4]
        l1_to_l4 = transform.identity_matrix()
        for t in trans:
            r_matrix = transform.euler_matrix(t[3], t[4], t[5])  # 4x4
            t_vector = t[:3]
            r_matrix[:3, 3] = t_vector
            # note that here we use r_matrix right product l1_to_l7
            l1_to_l4 = np.dot(l1_to_l4, r_matrix)
        print(l1_to_l4)

        trans = [l1_to_l2, l2_to_l3, l3_to_l4, l4_to_l5, l5_to_l6]
        l1_to_l6 = transform.identity_matrix()
        for t in trans:
            r_matrix = transform.euler_matrix(t[3], t[4], t[5])  # 4x4
            t_vector = t[:3]
            r_matrix[:3, 3] = t_vector
            # note that here we use r_matrix right product l1_to_l7
            l1_to_l6 = np.dot(l1_to_l6, r_matrix)
        print(l1_to_l6)


if __name__ == '__main__':
    unittest.main()
