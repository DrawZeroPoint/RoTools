# server

The server provides commonly used services for control or get information from a robot.

| Service                       | Function                                                                                                    |
|-------------------------------|-------------------------------------------------------------------------------------------------------------|
| get\_all\_group\_names        | Get joint names for all planning groups                                                                     |
| get\_active\_group\_names     | Get joint names for active planning group                                                                   |
| get\_group\_pose              | Get eef pose of the group in base frame                                                                     |
| get\_group\_joint\_states     | Get joint states of the group                                                                               |
| execute\_group\_position      | Execute a position for the group, the position could be absolute or relative                                |
| execute\_group\_shift         | Execute a shift for the group, the shift is absolute, could be applied to x y z roll pitch yaw individually |
| execute\_group\_pose          | Execute a pose for the group, the pose could be absolute or relative                                        |
| execute\_group\_joint\_states | Execute a series of joint states for the group, the states must be arranged as the order of joint names     |
| execute\_group\_named\_states | Execute a named states for the group, like ‘home’                                                           |
| execute\_group\_plan          | Execute a trajectory plan for the group, could be absolute or relative                                      |
