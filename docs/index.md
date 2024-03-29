# RoTools

RoTools is an all-in-one ROS package for high-level robotic task scheduling, visual perception, path planning,
simulation, and direct-/tele-manipulation control. It leverages BehaviorTree to deliver fast task construction and
coordination, and provides various utilities to bridge the gap between real/simulated robot and the high level task
scheduler.

The packages compose of two components: roport and rotools.

## roport

This module provides the application level entrance ports of the RoTools package. It is a middleware that allows using
rotools cpp/python interfaces in ROS environment. It provides:

### Python based

- `MoveIt Python Server` for controlling the robot's single kinematic chain using
  MoveIt's Python interface.
- `Sensing Server` that bridges the perception modules outside the ROS environment
  (like those run in Python3) to ROS via HTTP.
- `Planner Server` bridges ROS modules with the planning algorithm outside the ROS
  environment (running in Python3 or on another server on the local network) via HTTP. This server is designed for
  online control, that given the current state, it will query the algorithm for the next state. For now, the states are
  Cartesian poses.
- `Snapshot Server` enables recording various types of ROS msgs to files.
- `Websocket Client` enables transmitting ROS msgs among two PCs through ROS Bridge.
- `Xsens Server` converting the live stream from Xsens MVN Awinda motion capture suit to
  ROS pose messages.
- `OptiTrack Client` converting streams from OptiTrack socket server to pose msgs.
- Hardware/Simulation interfaces adapted for a variety types of robots.

### CPP based

- `MoveIt CPP Server` simultaneously controlling multiple kinematic chains of a robot
  with MoveIt's action interface.
- `Path Planning Interface` utilizes Humanoid Path Planner to plan whole-body collision
  free paths for high-dof robots.
- `Msg Converter` converts msg from one type to another, be able to modify the name field,
  and enables smoothly start the control of certain joint groups.
- `Task Scheduler` using behavior tree for task scheduling. A bunch of general purpose
  services and a node are provided for building the task tree fast.

## rotools

This Python package hosted under `src/` is a versatile robotic toolbox aimed for fast prototyping. It includes
foundational modules for robotic problems including path & trajectory planning, kinematics & dynamics calculation,
sensing, transformation calculation, simulation, and so forth.
