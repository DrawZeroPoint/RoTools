#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest

import numpy as np

from rotools.robot.serial.predefined_models import *
import rotools.robot.serial.model as serial_model

import rotools.utility.transform as transform
import rotools.utility.common as common


class Test(unittest.TestCase):

    def test_panda_model_fk(self):
        mdh, q_limits = panda_mdh_model()
        panda_mdh = serial_model.RobotModel.get_model_from_mdh(mdh)
        panda_mdh.q_limits = q_limits

        poe, q_limits = panda_poe_model()
        panda_poe = serial_model.RobotModel.get_model_from_poe(poe)
        panda_poe.q_limits = q_limits

        q = panda_mdh.random_valid_q()
        pose_mdh = panda_mdh.fk(q)
        pose_poe = panda_poe.fk(q)
        # self.assertTrue(common.all_close(pose_mdh, pose_poe, tolerance=1e-2))

    def test_ur10e_model_fk(self):
        mdh, q_limits = ur10e_mdh_model()
        ur10e_mdh = serial_model.RobotModel.get_model_from_mdh(mdh)
        ur10e_mdh.q_limits = q_limits

        poe, q_limits = ur10e_poe_model()
        ur10e_poe = serial_model.RobotModel.get_model_from_poe(poe)
        ur10e_poe.q_limits = q_limits

        q = ur10e_mdh.random_valid_q()
        pose_mdh = ur10e_mdh.fk(q)
        pose_poe = ur10e_poe.fk(q)
        self.assertTrue(common.all_close(pose_mdh, pose_poe))

    def test_ur10e_model_ik(self):
        # mdh, q_limits = ur10e_mdh_model()
        # ur10e_mdh = serial_model.RobotModel.get_model_from_mdh(mdh)
        # ur10e_mdh.q_limits = q_limits
        #
        # q = ur10e_mdh.random_valid_q()
        # pose = ur10e_mdh.fk(q)
        # q_ik = ur10e_mdh.ik_in_space(pose, ur10e_mdh.q0)
        # self.assertTrue(common.all_close(q, q_ik))

        poe, q_limits = ur10e_poe_model()
        ur10e_poe = serial_model.RobotModel.get_model_from_poe(poe)
        ur10e_poe.q_limits = q_limits

        q = ur10e_poe.random_valid_q()
        pose = ur10e_poe.fk(q)
        q_ik = ur10e_poe.ik_in_space(pose, q)
        self.assertTrue(common.all_close(q, q_ik))

    # def test_create_walker_arm_model(self):
    #     """Create a product of exponential model from urdf file.
    #     The following data is from walker.urdf, the left arm
    #
    #     """
    #     # [x, y, z, r, p, y] from urdf
    #     # we let the static base frame {s} of the arm coincide with joint 1,
    #     # hence the transformation from base link to {s} is fixed and equal to base_to_l1
    #     base_to_l1 = np.array([0, 0.24142, 0.018386, -1.931E-07, 1.3073, 1.5708])
    #     base_to_l1_matrix = transform.euler_matrix(base_to_l1[3], base_to_l1[4], base_to_l1[5])
    #     base_to_l1_matrix[:3, 3] = base_to_l1[:3]
    #     print('matrix from base to static\n', np.array2string(base_to_l1_matrix, separator=', '))
    #
    #     s_to_l1 = np.array([0, 0, 0, 0, 0, 0])
    #     l1_to_l2 = np.array([0, 0, 0, -1.5708, 1.5708, 0])
    #     l2_to_l3 = np.array([-0.00064368, -0.2285, 0, 1.5708, 6.1653E-11, -2.7756E-17])
    #     l3_to_l4 = np.array([0.025422, 0, 0, -1.5708, -0.046339, 3.1416])
    #     l4_to_l5 = np.array([0.024818, -0.22241, 0, -1.5708, -1.5708, -3.1416])
    #     l5_to_l6 = np.array([0, 0, 0, 1.5708, -1.5708, 0])
    #     l6_to_l7 = np.array([0, 0, 0, 1.5708, 3.0389E-17, 6.4236E-24])
    #
    #     trans = [s_to_l1, l1_to_l2, l2_to_l3, l3_to_l4, l4_to_l5, l5_to_l6, l6_to_l7]
    #     M = transform.identity_matrix()  # home matrix
    #     for t in trans:
    #         r_matrix = transform.euler_matrix(t[3], t[4], t[5])  # 4x4
    #         t_vector = t[:3]
    #         r_matrix[:3, 3] = t_vector
    #         # note that here we use r_matrix right product l1_to_l7
    #         M = np.dot(M, r_matrix)
    #     print('The M matrix for walker`s left arm is\n', np.array2string(M, separator=', '))
    #
    #     trans = [l1_to_l2, l2_to_l3, l3_to_l4]
    #     l1_to_l4 = transform.identity_matrix()
    #     for t in trans:
    #         r_matrix = transform.euler_matrix(t[3], t[4], t[5])  # 4x4
    #         t_vector = t[:3]
    #         r_matrix[:3, 3] = t_vector
    #         # note that here we use r_matrix right product l1_to_l7
    #         l1_to_l4 = np.dot(l1_to_l4, r_matrix)
    #     print(l1_to_l4)
    #
    #     trans = [l1_to_l2, l2_to_l3, l3_to_l4, l4_to_l5, l5_to_l6]
    #     l1_to_l6 = transform.identity_matrix()
    #     for t in trans:
    #         r_matrix = transform.euler_matrix(t[3], t[4], t[5])  # 4x4
    #         t_vector = t[:3]
    #         r_matrix[:3, 3] = t_vector
    #         # note that here we use r_matrix right product l1_to_l7
    #         l1_to_l6 = np.dot(l1_to_l6, r_matrix)
    #     print(l1_to_l6)


if __name__ == '__main__':
    unittest.main()
