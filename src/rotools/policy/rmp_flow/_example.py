from _root import RMPRoot
from _leaf import CollisionAvoidance, GoalAttractorUni

import numpy as np
from numpy.linalg import norm
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.patches as patches

# ---------------------------------------------
# build the rmp tree
x_g = np.array([-3, 1, 1])  # position of the goal
x_o = np.array([-1, 0, 0])  # center of the sphere obstacle
r_o = 1.3  # radius of the obstacle

r = RMPRoot('root')
leaf1 = CollisionAvoidance('collision_avoidance', r, None, c=x_o, R=r_o, epsilon=0.2)
leaf2 = GoalAttractorUni('goal_attractor', r, x_g)


x = np.array([2.5, -2, -1])  # init position
x_dot = np.array([0, 0, 0])  # init velocity

state_0 = np.concatenate((x, x_dot), axis=None)
print(state_0.reshape(2, -1))


# dynamics
def dynamics(t, state):
    # the function to solve, iteratively used in solve_ivp
    state = state.reshape(2, -1)
    x = state[0]
    x_dot = state[1]
    x_ddot = r.solve(x, x_dot)  # RMP is used here!
    state_dot = np.concatenate((x_dot, x_ddot), axis=None)
    return state_dot


# solve the diff eq
sol = solve_ivp(dynamics, [0, 140], state_0)
# ---------------------------------------------

# --------------------------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

u = np.linspace(0, 2 * np.pi, 30)
v = np.linspace(0, np.pi, 30)
ob_x = r_o * np.outer(np.cos(u), np.sin(v)) + x_o[0]
ob_y = r_o * np.outer(np.sin(u), np.sin(v)) + x_o[1]
ob_z = r_o * np.outer(np.ones(np.size(u)), np.cos(v)) + x_o[2]

ax.plot_surface(ob_x, ob_y, ob_z, rstride=1, cstride=1, cmap='rainbow', edgecolor='none')

# draw trajectory
ax.plot3D(sol.y[0], sol.y[1], sol.y[2], 'gray')

# draw starting and ending point
se = np.stack((x, x_g), axis=-1)
ax.scatter3D(se[0], se[1], se[2], c='gr')

fig_vel = plt.figure()
ax_vel = fig_vel.add_subplot(111)

ax_vel.plot(sol.t, sol.y[3], color='red', linewidth=3)
ax_vel.plot(sol.t, sol.y[4], color='green', linewidth=3)
ax_vel.plot(sol.t, sol.y[5], color='blue', linewidth=3)

plt.show()
# --------------------------------------------
