# study-moveit
<p align = "center">
    <img src="assets/rviz_gazebo_icub.png"/>
</p>

## Overview

This repository contains the current state of the ROS2 package for using iCub with MoveIt.

## Assumptions

This package was generated and tested with `ros humble` on WSL2 running `Ubuntu 22.04`.

## Prerequisites 

In order to run this package, follow the [MoveIt 2 instruction](https://moveit.ros.org/install-moveit2/source/) to setup your ROS2 workspace from source.

## Install

In your ROS2 workspace, clone the repo:

```shell
cd <ros2_ws>/src
git clone https://github.com/icub-tech-iit/study-moveit
```
and then build the environment:

```shell
cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Launch

The `icub.launch.py` launch file allows to start up a set of nodes at the same time. In particular, it spawns the iCub model on both `gazebo` and `rviz` environments, publishes the state of the robot to `tf2` and starts the `ros2_control` node.

```shell
source install/setup.bash
ros2 launch icub_moveit icub.launch.py
```

`icub_moveit.launch.py`, instead, includes the [`move_group`](https://moveit.picknik.ai/main/doc/concepts/move_group.html) node which provides a set of ROS actions and services.

```shell
source install/setup.bash
ros2 launch icub_moveit icub_moveit.launch.py
```