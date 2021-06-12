import numpy as np

from typing import Sized
from attr import attrs, attrib
from scipy.integrate import solve_ivp

from _root import RMPRoot
from _leaf import GoalAttractorUni, CollisionAvoidance


@attrs
class RMPPlanner(Sized):
    """Planner class using Riemannian Motion Policy."""

    def __len__(self):
        return self.dim

    dim = attrib(type=int)
    _root = None  # attrib(factory=lambda: RMPRoot('root'))

    def _dynamics(self, t, state):
        """the function to solve, iteratively used in solve_ivp.

        :param t:
        :param state: ndarray 2*dim [x1, x2, ..., xdot1, xdot2, ...]
        :return:
        """
        state = state.reshape(2, -1)
        x = state[0]
        x_dot = state[1]
        x_ddot = self._root.solve(x, x_dot)  # RMP is used here!
        state_dot = np.concatenate((x_dot, x_ddot), axis=None)
        return state_dot

    def plan(self, state, goal, obstacle=None, span=None):
        """Given initial robot state (x xdot), where x is the eef position and xdot=0,
        along with goal position, plan a trajectory from initial state to the goal.

        :param state: ndarray initial state as [x1, ..., xn, xdot1, ..., xdotn], n the dim of coord
        :param goal: ndarray target state [gx1, gx2, ..., gxn]
        :param obstacle: ndarray [x, y, z, r] obstacle center (x,y,z) and radius r
        :param span: the total execution time interval in second
        :return: Trajectory positions, velocities, timestamps
        """
        if span is None:
            span = [0, 5]  # default plan a 5s trajectory

        # For now, the root and leaf need to be explicitly initialized, otherwise
        # some temporary state will make the planning go wrong TODO figure the reason
        self._root = RMPRoot('root')
        leaf_goal = GoalAttractorUni('goal_attractor', self._root, goal, gain=1)
        if obstacle is not None:
            leaf_obs = CollisionAvoidance('collision_avoidance', self._root,
                                          None, c=obstacle[:3], R=obstacle[3], epsilon=0.2)
        sol = solve_ivp(self._dynamics, span, state)
        if sol:
            positions = sol.y[:self.dim, :]  # shape (dof, N), N is the way point number
            velocities = sol.y[self.dim:, :]
            timestamps = sol.t
            return timestamps, positions, velocities  # accelerations
        else:
            return None
