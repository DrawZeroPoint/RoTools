# RoTools

RoTools is an all-in-one ROS package for high-level robotic task scheduling,
visual perception, direct and tele-manipulation control. It leverages BehaviorTree to
deliver fast task construction and coordination, and uses MoveIt
and hardware_interface to bridge the gap between real/simulated
robot and the high level task scheduler.

## Contents

The packages compose of two components: roport and rotools.

### roport

This module provides the entrance ports of the RoTools package. It is a middleware that allows using 
rotools python interface in ROS environment. It provides: 

- [MoveIt server](scripts/roport_moveit_server.py) for controlling the robot's single kinematic chain.
- [control server](src/roport_control_server.cpp) simultaneously controlling multiple kinematic chains of the robot.
- [sensing server](scripts/roport_sensing_server.py) that bridges the perception modules outside the ROS environment 
  (like those run in Python3) to ROS via HTTP.
- [task scheduler](src/roport_task_scheduler.cpp) using behavior tree for task scheduling. 
  A bunch of general purpose services are provided for building the task map fast.
- [xsens server](scripts/roport_xsens_server.py) converting the live stream from Xsens's MVN Awinda motion capture
  suit to ROS pose messages. 
- Hardware/Simulation interfaces adapted for a variety types of robots.

### rotools

This Python package hosted under `src/` is a versatile robotic tool-box aimed for fast prototyping.
It includes foundational modules for robotic problems including path & trajectory planning, kinematics
& dynamics calculation, sensing, transformation calculation, and so forth.

## Prerequisite

### System

Well tested on Ubuntu 18.04 with Python 2.7 and ROS Melodic. ROS Indigo on Ubuntu 14.04 is not supported.
May also work for other version combinations.

### Officially supported robots

- Walker from UBTech
- Curiosity from SMART Lab CUHK

### Dependence

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

## Usage

### Use with MoveIt!

The pipeline of using RoTools for a particular robot involves:

Create a description package for the robot you want to use, you can refer to 
[curiosity](https://github.com/DrawZeroPoint/curiosity) or walker repo. 
This may involve:

   1. Create a launch file named `<robot>_moveit.launch` for bringing up the MoveIt interfaces,
   hardware interfaces, and optionally a converter for remapping standard measure/control topics
   to those used by the real/simulated robot. 
   
   2. Create a launch file named `<robot>_roport.launch` for bridging RoPort MoveIt server,
   which essentially is a middleware between the MoveIt interface and the task scheduler.
   It could disassemble various type of movements for MoveIt to plan and execute.
   
   3. Arrange the tasks with BT using [Groot](https://github.com/BehaviorTree/Groot). 
   Note that a predefined BT node package (palette.xml) has been provided,
   you can load that into Groot for a quick start. After generating the xml file describing the task,
   feed it as a parameter of the `<robot>_task.launch`.
   
After finishing these preparations, you can run the demo by

1. Start the simulator or connect to the real robot. Do not forget starting `roscore` when using the simulator.
2. Launch `<robot>_moveit.launch`.
3. Launch `<robot>_roport.launch`.
4. Launch `<robot>_task.launch`.

### Use with CartesI/O

CartesI/O is a whole-body motion planning software developed in [ADVR, Istituto Italiano di Tecnologia](https://advr.iit.it/). 
It has not been added to the ROS distribution and hence [install](https://advrhumanoids.github.io/CartesianInterface/index.html) 
it is needed for proceeding the next steps.

1. Prepare the URDF file and SRDF file of the robot you use. 
   [Certain rules](https://advrhumanoids.github.io/CartesianInterface/quickstart.html#setting-up-the-robot-description) 
   should be followed.
   
2. Create a launch file with the name <robot>_cartesio.launch.
   
After the preparations, you can

1. launch the file by:
   ```shell
   roslaunch roport <robot>_cartesio.launch
   ```
   This will show a RViz window if you did not set gui:=false. See the explanation of the arguments by launch with 
   `--ros-args`.

2. In RViz, right-click the interactive marker (IM) in scene, choose `continous control`, then you can control
   the robot by dragging the IM.
   
### Use with Xsens motion capture stream

#### Launch process

The IMU based motion capture suit provided by [Xsens](https://www.xsens.com/) could be used as a direct tele-operation
input source for controlling the robot. RoTools enables converting the motion capture stream sent via UDP to ROS messages.
To establish the conversion:

1. Configure the MVN Animate/Analyze software, enable transmitting the live stream to the receiving machine's IP 
   (the machine running RoTools). The default port number is 9763. You need to get a license for the MVN software to use 
   the streaming function.
   
2. Make sure the publisher and receiver machines are in the same local network. From now on,
   we assume the subscriber's network as:
   
   ```text
   inet 192.168.13.234  netmask 255.255.255.0  broadcast 192.168.13.255
   ```
   While the publisher is:
   ```
   inet 192.168.13.102  netmask 255.255.255.0  broadcast 192.168.13.255
   ```
   
   The `ufw` of the receiver should be configured to allow incoming access (note 13 is variable while 0/24 is fixed):
   ```shell
   sudo ufw allow from 192.168.13.0/24
   ```
   
3. Launch the xsens server
   ```shell
    roslaunch roport roport_xsens.launch udp_ip:=192.168.13.234
   ```
   **Note that the server will be initialized after it receiving the first datagram from the stream,
     so the server should be started after streaming.**
   
   Here, `192.168.13.234` is the local IP address of the receiver machine running this file. 
   Note that you cannot use an IP like `127.0.0.1`.
   
#### Converted topics

By default, the server will publish poses on topics: `/xsens/all_poses`, `/xsens/body_poses`, `/xsens/left_tcp`,
`/xsens/right_tcp`, `/xsens/left_sole`, `/xsens/right_sole`. 
The former two are in `PoseArray` format, the latter ones are in `PoseStamped`.
`body_poses` are the poses of body segments without hand segments; `left_tcp` is the left palm pose; 
`right_tcp` is the right palm pose. `left_sole` is the left palm pose; `right_sole` is the right palm pose.
When standing in N pose, the TCP poses and sole poses is illustrated as:



You can use `remap` in the launch file to remap some of these to other topic names.

The server will also publish hand joint states on topics: `/xsens/left_hand_js`, and `/xsens/right_hand_js`.
The joint states for each hand compose of 10 values ranging from 0 to 1, where 0 is fully stretch, 1 is fully bend.

| 1        | 2        | 3        | 4        | 5         | 6         | 7       | 8       | 9        | 10       |
|----------|----------|----------|----------|-----------|-----------|---------|---------|----------|----------|
| thumb_j1 | thumb_j2 | index_j1 | index_j2 | middle_j1 | middle_j2 | ring_j1 | ring_j2 | pinky_j1 | pinky_j2 |

*Note: j1 is the joint between metacarpals (parent) and proximal phalanges (child); j2 is the angle between proximal 
phalanges and intermediate/distal phalanges (distal is used only for the thumb)*

These poses all reference to the reference_frame, which could be 'Pelvis' or 'T8'. You can set the 
reference frame in the launch file. If the frame name is not within these two, a given one will be used.
If empty string is given, the default one `world` will be used.

#### State switch

For safety concern, you need to activate the conversion by:

Press the `space` key on the keyboard. This will switch the status from `deactive` to `active`.
To change back, press `space` again. You can press any other keys to check the current status, but not change it.

Alternatively, you can use:

```shell script
rosservice call /xsens/enable "data: true"
```
The initial state is in deactivate state, to disable conversion, set `data` as false.

## API reference

### roport_robot_interface

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


## Coding Guide

### The services

#### Naming

- The name should obey the CamelCase convention. Each word in the name should avoid 
meaningless abbreviation, for example, Object is better than Obj.

- The first word should only be `Get`, `Sense`, or `Execute`, where `Get` means
retrieve some information from within the software, such as all names of planning
groups, or robot joint states; `Sense` means get information from outside environment,
such as get the pose of an object; and `Execute` means interact with the environment,
such as open a switch, move the robot arm, or add a collision object into the planning scene.

- If the second word is `Group`, it means the service operates on a planning group in MoveIt.
Similarly, if the word is `All`, it means applying to all planning groups. If the context is
clear, such as a collision object could only be added to a group, the `Group` could be omitted.

- For the third and the following words, `Plan/Plans` means a , `Pose/Poses` means the 
6-DoF Cartesian pose in workspace, `JointStates` means the joint values in C-space.

#### Content

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


## Note

### Simultaneously execution

The python version MoveItServer do not support simultaneously execution. To
perform such function, you need to directly send goal to the controller client.
The `roport_control_server` serves for this purpose.