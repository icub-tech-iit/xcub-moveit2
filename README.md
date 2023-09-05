study-moveit
=============

<div align = "center">
    <img src="assets/rviz_gazebo_icub.png"/>
</div>


## Overview

This repository contains the current state of the ROS2 package for using iCub with MoveIt.

## Assumptions

These packages were generated and tested with `ros humble` on WSL2 running `Ubuntu 22.04`.

## Prerequisites

First of all, install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your machine and configure your [ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#configuring-environment). Then, follow the [MoveIt 2 binary install](https://moveit.ros.org/install-moveit2/binary/) guide to build all the necessary dependencies.

Inside your ROS 2 workspace, it is mandatory to install `yarp-devices-ros2` to use custom ROS messages and services defined in [yarp_control_msgs](https://github.com/robotology/yarp-devices-ros2/tree/master/ros2_interfaces_ws/src/yarp_control_msgs):

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

## Install

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

## Launch

The `icub.launch.py` launch file allows to start up a set of nodes at the same time. In particular, it spawns the iCub model on both `gazebo` and `rviz` environments, publishes the state of the robot to `tf2`, starts the `ros2_control` node and the [move_group](https://moveit.picknik.ai/main/doc/concepts/move_group.html) node, which provides a set of ROS actions and services for using MoveIt 2 with your robot.

```shell
source install/setup.bash
ros2 launch icub_moveit icub.launch.py
```
