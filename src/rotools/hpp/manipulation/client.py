#!/usr/bin/env python
from __future__ import print_function

import nav_msgs.msg
import rospy
import rostopic
import sensor_msgs.msg

from sensor_msgs.msg import JointState, Image, CompressedImage
from nav_msgs.msg import Odometry

from roport.srv import *

import rotools.hpp.manipulation.interface as interface


class HPPManipulationClient(object):
    """This client uses the HPP manipulation interface to connect to the HPP cobra server and provide some
    handy services for manipulation plannning.
    """

    def __init__(self, kwargs):
        super(HPPManipulationClient, self).__init__()

        self.interface = interface.HPPManipulationInterface(**kwargs)

        self._srv_execute_manipulation_planning = rospy.Service('execute_manipulation_planning',
                                                                ExecuteManipulationPlanning,
                                                                self.execute_manipulation_planning_handle)

    def execute_manipulation_planning_handle(self, req):
        resp = ExecuteManipulationPlanningResponse()
        return resp
