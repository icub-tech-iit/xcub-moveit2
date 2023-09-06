study-moveit
=============

<div align = "center">
    <img src="assets/rviz_gazebo_icub.png"/>
</div>


## Overview

This repository contains the current state of the ROS2 package for using iCub with MoveIt.

## Assumptions

These packages were generated and tested with `ros humble` on a `Ubuntu 22.04` machine.

## Prerequisites

First of all, install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your machine and configure your [ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#configuring-environment). Then, follow the [MoveIt 2 install guide](https://moveit.ros.org/install-moveit2/binary/) to build all the necessary dependencies.

Inside your ROS2 workspace, it is mandatory to install `yarp-devices-ros2` to use custom ROS messages and services defined in [yarp_control_msgs](https://github.com/robotology/yarp-devices-ros2/tree/master/ros2_interfaces_ws/src/yarp_control_msgs):

```shell
cd ~/<ros2_ws>/src

# Clone the repository 
git clone https://github.com/robotology/yarp-devices-ros2

# Build ROS msgs and compile the colcon workspace
cd yarp-devices-ros2/ros2_interfaces_ws
colcon build
source install/setup.bash

# Compile yarp-devices-ros2
cd ~/<ros2_ws>/src/yarp-devices-ros2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<install_prefix>
make
make install
```

Finally, [TRAC-IK](https://traclabs.com/projects/trac-ik/) is chosen as inverse kinematics solver. It is more accurate and faster when dealing with complex kinematic chains with respect to KDL Kinematics, which represents the standard for MoveIt 2. To install it inside your ROS2  workspace:

```shell
cd ~/<ros2_ws>/src
source /opt/ros/humble/setup.bash

# Clone the repository inside your ros2 workspace and build it
git clone https://bitbucket.org/traclabs/trac_ik.git -b rolling-devel
cd trac_ik
colcon build
source install/setup.bash
```

## Install

This section describes how to install and build the packages contained in this repository.

In your ROS2 workspace, clone the repo:

```shell
cd ~/<ros2_ws>/src
git clone https://github.com/icub-tech-iit/study-moveit
```

and then build the environment:

```shell
cd ~/<ros2_ws>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Package description

This section aims to give a brief description of what each package contains.

### icub_controller

This package contains `icub_controller` plugin that is used in [ros2_control](https://control.ros.org/master/index.html) framework. It includes:

- a `position state interface` used to read the position of each joint;
- a `velocity state interface` used to read the velocity of each joint;
- a `position command interface` used to forward the desired position to the joints.

### icub_moveit

This package contains the `icub.launch.py` launch file that allows to start up a set of nodes at the same time. In particular, it spawns the iCub model on both `gazebo` and `rviz` environments, publishes the state of the robot to `tf2` topic, spawns the `ros2_control` nodes and the [move_group](https://moveit.picknik.ai/main/doc/concepts/move_group.html) one, which provides some ROS actions and services for using MoveIt 2 with your robot. To see if everything works, you can try to run:

```shell
source install/setup.bash
ros2 launch icub_moveit icub.launch.py
```


### icub_moveit_config

This package contains the configuration files to work with MoveIt2. In particular, each of the parts of iCub robot (head, left_arm, right_arm, torso, left_leg and right_leg) are defined in terms of `planning group`, and for each of them a ros2_control of type `FollowJointTrajectory` is set.
