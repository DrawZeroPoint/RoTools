#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function

import unittest

import numpy as np

import rotools.robot.serial.predefined_models as predefined
import rotools.robot.serial.model as serial_model


class Test(unittest.TestCase):

    def test_ur5_fk(self):

        q = [0, -np.pi/2, 0, 0, np.pi/2, 0]

        ur5_poe_param = predefined.ur5_poe_model()
        ur5_poe = serial_model.RobotModel.from_poe_parameters(ur5_poe_param)

        pose_poe = ur5_poe.fk(q)
        print('ur5 poe \n', pose_poe)

    def test_ur10e_fk(self):

        q = [np.pi/2, 0, 0, 0, 0, 0]

        ur10e_poe_param = predefined.ur10e_poe_model()
        ur10e_poe = serial_model.RobotModel.from_poe_parameters(ur10e_poe_param)

        pose_poe = ur10e_poe.fk(q)
        print('ur10e poe \n', pose_poe)

        ur10e_mdh_param = predefined.ur10e_mdh_model()
        ur10e_mdh = serial_model.RobotModel.from_mdh_parameters(ur10e_mdh_param)

        pose_mdh = ur10e_mdh.fk(q)
        print('ur10e mdh \n', pose_mdh)


if __name__ == '__main__':
    unittest.main()
