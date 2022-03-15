import rospy
import numpy as np

from dmp.msg import *
from dmp.srv import *


class DMPInterface(object):
    """

    """

    def __init__(self, n_dim, k, **kwargs):
        """
        :param n_dim: int
        :param k:
        """
        super(DMPInterface, self).__init__()
        self.n_dim = n_dim
        self.k = k
        self.d = 2.0 * np.sqrt(k)

        self.dmp_list = None
        self.tau = None

        self._set_client = rospy.ServiceProxy('/set_active_dmp', SetActiveDMP)
        self._learning_client = rospy.ServiceProxy('/learn_dmp_from_demo', LearnDMPFromDemo)
        self._get_plan_client = rospy.ServiceProxy('/get_dmp_plan', GetDMPPlan)

    def learning(self, demo):
        """

        :param demo: dict Demo should contain 2 lists: points and times, wherein points
                     compose of 2 lists, i.e., positions and velocities.
        """
        req = LearnDMPFromDemoRequest()
        dmp_trajectory = DMPTraj()
        points = demo['points']
        for point in points:
            dmp_point = DMPPoint()
            dmp_point.positions = point[0]
            dmp_point.velocities = point[1]
            assert len(dmp_point.positions) == self.n_dim and len(dmp_point.velocities) == self.n_dim
            dmp_trajectory.points.append(dmp_point)
        dmp_trajectory.times = demo['times']
        req.demo = dmp_trajectory

        req.k_gains = [self.k] * self.n_dim
        req.d_gains = [self.d] * self.n_dim
        req.num_bases = 1000
        resp = self._learning_client.call(req)
        self.dmp_list = resp.dmp_list
        self.tau = resp.tau

    def set_active_dmp(self):
        req = SetActiveDMPRequest()
        if self.dmp_list is None:
            raise ValueError
        req.dmp_list = self.dmp_list
        resp = self._set_client.call(req)
        if resp.success:
            rospy.loginfo('DMP set')
        else:
            rospy.logerr('DMP not set!')

    def get_dmp_plan(self, x_0, x_dot_0, goal, goal_thresh=0.001,
                     dt=1.0, t_0=0, seg_length=-1, integrate_iter=5):
        if self.tau is None:
            raise ValueError
        req = GetDMPPlanRequest()
        req.x_0 = x_0
        req.x_dot_0 = x_dot_0
        req.goal = goal
        req.goal_thresh = [goal_thresh] * self.n_dim
        req.dt = dt
        req.t_0 = t_0
        req.tau = self.tau
        req.seg_length = seg_length
        req.integrate_iter = integrate_iter
        resp = self._get_plan_client.call(req)
        # resp = GetDMPPlanResponse()
        positions = []
        for point in resp.plan.points:
            positions.append(point.positions)
        return np.array(positions)
