#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function

import os.path
import unittest

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from rotools.policy.dmp.interface import DMPInterface


class Test(unittest.TestCase):

    def test_dmp(self):
        root_dir = os.path.dirname(os.path.realpath(__file__))
        print(root_dir)
        test_file = os.path.join(root_dir, 'test_data/0-4.npy')

        # data is a 3d array, where the first dimension is the demonstrations,
        # the second dimension is the way points, and the last is the features.
        data = np.load(test_file)
        print(data.shape)

        # FIXME Use multiple trajectories instead of the first
        demo_positions = data
        velocities = np.gradient(demo_positions, axis=0)
        points = []
        for p, v in zip(demo_positions.tolist(), velocities.tolist()):
            point = [p, v]
            points.append(point)

        dt = 1.0
        times = np.arange(0, dt * len(points), dt).tolist()
        demo = {'points': points, 'times': times}

        dmp_interface = DMPInterface(3, 1000)
        dmp_interface.learning(demo)
        dmp_interface.set_active_dmp()
        positions = dmp_interface.get_dmp_plan(
            x_0=[0., 0.16, 0],
            x_dot_0=[0, 0, 0],
            goal=[0, 0, 0]
        )
        save_path = os.path.join(root_dir, 'test_data/out.npy')
        np.save(save_path, positions)

        ax = plt.axes(projection='3d')
        ax.set_proj_type('ortho')

        def visualize_rollout(rollout, show=True):
            i = np.arange(0, rollout.shape[0])  # way point id
            x = rollout[:, 0]
            y = rollout[:, 1]
            z = rollout[:, 2]
            ax.plot3D(x, y, z, 'y')
            ax.scatter3D(x, y, z, s=9, c=i, cmap='rainbow')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            if show:
                plt.show()

        visualize_rollout(demo_positions, False)
        visualize_rollout(positions)


if __name__ == '__main__':
    unittest.main()
