# RoTools

RoTools is a all-in-one ROS package for high-level robotic task scheduling,
visual perception, and robotic manipulation control. It leverages BehaviorTree to
deliver fast task map construction and coordination, and uses MoveIt! 
and hardware_interface to bridge the gap between real/simulated
robot and the high level task scheduler.

The packages is composed by 2 components: roport and rotools.

## roport

This module provides the entrance ports of the RoTools package. It is a middleware that allows using 
rotools python interface in ROS environment. It provides: 

- A [Python MoveIt server](scripts/roport_moveit_server.py) for controlling the robot's single kinematic chain.
- A [C++ control server](src/roport_control_server.cpp) simultaneously controlling multiple kinematic chains of the robot.
- A [Sensing server](scripts/roport_sensing_server.py) that bridges the perception modules outside the ROS environment 
  (like those run in Python3) to ROS via HTTP.
- A BehaviorTree node written in C++ for task scheduling. A bunch of general purpose services are provided for building 
  the task map fast.
- Hardware/Simulation interfaces adapted for a variety types of robots.

## rotools

This Python package hosted under `src/` is a versatile robotic tool-box aimed for fast prototyping.
It includes foundational modules for robotic problems including path & trajectory planning, kinematics
& dynamics calculation, sensing, transformation calculation, and so forth.

# Prerequisite

## System

Well tested on Ubuntu 18.04, Python 2.7, and ROS Melodic. ROS Indigo on Ubuntu 14.04 is not supported.
May also work for other version combinations.

## Officially supported robots

- Walker from UBTech
- Curiosity from ULTRA Lab CUHK

## Dependence

#### Noetic

```shell script
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3 ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers \
ros-$ROS_DISTRO-webots-ros ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-std-srvs ros-$ROS_DISTRO-trac-ik-lib \
ros-$ROS_DISTRO-eigen-conversions
```

#### Melodic

```shell script
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3 ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers \
ros-$ROS_DISTRO-webots-ros ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-std-srvs ros-$ROS_DISTRO-trac-ik-kinematics-plugin \
ros-$ROS_DISTRO-eigen-conversions
```

#### Indigo

Not supported.

# Usage

## Use with MoveIt!

The pipeline of using RoTools for a particular robot involves:

Create a description package for the robot you want to use, you can refer to 
[curiosity](https://github.com/DrawZeroPoint/curiosity) or walker repo. 
This may involve:

   1. Create a launch file named `<robot>_moveit.launch`
   for bringing up the MoveIt interfaces. 
   
   2. Create a launch file named `<robot>_roport.launch` for bridging RoPort
   with the MoveIt interface. 
   
   3. Arrange the tasks with BT using [Groot](https://github.com/BehaviorTree/Groot). 
   Note that a predefined BT node package (palette.xml) has been provided,
   you can load that into Groot for a quick start.
   
After finishing these preparations, you can run the demo by

1. Start the simulator or connect to the real robot. Do not forget starting `roscore`.
2. Launch MoveIt interface and hardware interface.
3. Launch the control servers (could be Python and/or C++ servers).
4. Launch the task.

## Use with CartesI/O

1. Prepare the URDF file and SRDF file of the robot you use. 
   [Certain rules](https://advrhumanoids.github.io/CartesianInterface/quickstart.html#setting-up-the-robot-description) should be followed.
   
2. Create a launch file with the name <robot>_cartesio.launch.
   
After the preparations, you can 

1. launch the file by:
   ```
   roslaunch roport <robot>_cartesio.launch
   ```
   This will show a RViz window if you did not set gui:=false

2. In RViz, right click the interactive marker (IM) in scene, choose `continous control`, then you can control
   the robot by dragging the IM.

# API reference

## roport_robot_interface

This interface inherits the `hardware_interface` in `ros_control`. It gets measured joint 
states via one or more topics from the real robot or the simulator. Meanwhile, it
sets the joint states commands down to the robot or the simulator via one or
more topics.

To properly use this, you need to set the joint states getter/setter topics 
in the launch file like this:

```
  <node pkg="roport" type="roport_robot_interface" name="roport_robot_interface" output="screen">
    <param name="joint_name_param_id" value="/walker/hardware_interface/joints"/>
    <rosparam param="measure_joint_states_id">
      [
      "/walker/leftLimb/joint_states",
      "/walker/rightLimb/joint_states",
      ]
    </rosparam>
    <rosparam param="control_joint_states_id">
      [
      "/walker/leftLimb/command",
      "/walker/rightLimb/command",
      ]
    </rosparam>
    <rosparam param="joint_states_group">
      [
        [
          "left_limb_j1", "left_limb_j2", "left_limb_j3", "left_limb_j4",
          "left_limb_j5", "left_limb_j6", "left_limb_j7"
        ],
        [
          "right_limb_j1", "right_limb_j2", "right_limb_j3", "right_limb_j4",
          "right_limb_j5", "right_limb_j6", "right_limb_j7"
        ]
      ]
    </rosparam>
  </node>>
``` 

The param `measure_joint_states_id` and `control_joint_states_id` must be set.
Note that these topic ids have leading `/`. Besides, the `joint_states_name_group`
should be set for each getter and setter topics. If it is not set, there should
 be only one getter and one setter topic, in this case, the algorithm will use
all names set in the parameter `joint_name_param_id`. 


# Coding Guide

## The services

### Naming

- The name should obey the CamelCase convention. Each word in the name should avoid 
meaningless abbreviation, for example, Object is better than Obj.

- The first word should only be `Get`, `Sense`, or `Execute`, where `Get` means
retrieve some information from within the software, such as all names of planning
groups, or robot joint states; `Sense` means get information from outside environment,
such as get the pose of a object; and `Execute` means interact with the environment,
such as open a switch, move the robot arm, or add a collision object into the planning scene.

- If the second word is `Group`, it means the service operates on a planning group in MoveIt.
Similarly, if the word is `All`, it means applying to all planning groups. If the context is
clear, such as a collision object could only be added to a group, the `Group` could be omitted.

- For the third and the following words, `Plan/Plans` means a , `Pose/Poses` means the 
6-DoF Cartesian pose in workspace, `JointStates` means the joint values in C-space.

### Content

Add the following comment on top of each .srv file:
Here `[opt]` stands for optional param, whose value 
could be not given. In this case, note that the default
value will be used.

```
# Brief explanation about the service

# Requests
# param [opt]: explanation
# ...

# Response
# param [opt]: explanation
# ...
```

The request of the service should always contain:

```
std_msgs/Header header
```

The response of the service should always contain:

```
uint8 SUCCEEDED=0
uint8 FAILED=1
uint8 result_status
```

For these parameters the comment could be omitted.


# Note

### Simultaneously execution

The python version MoveItServer do not support simultaneously execution. To
perform such function, you need to directly send goal to the controller client.
The `roport_control_server` serves for this purpose.