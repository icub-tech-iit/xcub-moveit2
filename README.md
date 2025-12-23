xcub-moveit2
=============

<div align = "center">
    <img src="assets/rviz_gazebo_icub.png"/>
</div>

## Maintainer

| | |
|:---:|:---:|
| [<img src="https://github.com/martinaxgloria.png" width="40">](https://github.com/martinaxgloria) | [@martinaxgloria](https://github.com/martinaxgloria) |

## Overview

This repository contains the current state of the ROS 2 packages for using iCub and ergoCub with MoveIt.

## Assumptions

These packages were generated and tested with `ROS 2 Humble Hawksbill` distro on a `Ubuntu 22.04` machine.

## Prerequisites and dependencies

### MoveIt

First of all, install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your machine and configure your [ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#configuring-environment). Then, follow the [MoveIt 2 install guide](https://moveit.ros.org/install-moveit2/binary/) to build all the necessary dependencies.

### yarp-devices-ros2

Moreover, it is mandatory to install `yarp-devices-ros2` on your machine to use custom ROS messages and services defined in [yarp_control_msgs](https://github.com/robotology/yarp-devices-ros2/tree/master/ros2_interfaces_ws/src/yarp_control_msgs). To do this, you can follow the [installation procedure](https://github.com/robotology/yarp-devices-ros2?tab=readme-ov-file#installation) described in the repository. Otherwise, starting from the distro [`v2024.11.0`](../sw_versioning_table/2024.11.0.md), it can be compiled within the robotology-superbuild by enabling the `ROBOTOLOGY_USES_ROS2` CMake option.

This repository contains some devices and custom ROS 2 interfaces with different purposes and, in particular for this application, it contains the possibility to control a `yarp-based` robot with ROS 2. To enable this feature, you have to add the `msgs_name` parameter in your configuration file that inizializes the device `controlBoard_nws_ros2` (usually it is called `all-mc_remapper_ros2.xml` and it can be found under `conf/wrappers/motorControl` directory), for example:

![msgs](assets/msgs_parameter.jpg)

### TRAC-IK

[TRAC-IK](https://traclabs.com/projects/trac-ik/) is chosen as inverse kinematics solver. It is more accurate and faster when dealing with complex kinematic chains with respect to KDL Kinematics, which represents the standard for MoveIt 2. 

It can be installed within your ROS 2 workspace:

```shell
cd ~/<ros2_ws>/src
source /opt/ros/humble/setup.bash

# Clone the repository inside your ros2 workspace and build it
git clone https://bitbucket.org/traclabs/trac_ik.git -b 2.0.0
cd trac_ik
colcon build
source install/setup.bash
```

> **_NOTE:_** If you don't want to source your ros humble setup file each time a new shell is open, you can add this command in your `.bashrc`:
> ```shell
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> ```

### gz-sim-yarp-plugins

If you plan to use `xcub-moveit2` in a simulated environment, you need `gz-sim-yarp-plugins` to be installed on your machine. [`gz-sim-yarp-plugins`](https://github.com/robotology/gz-sim-yarp-plugins) is part of the `robotology-superbuild` and it could be compiled within this repository by enabling the `ROBOTOLOGY_USES_GZ` CMake option. 

### Further ros2 packages

Finally, install the following further ros2 packages since the project depends on them:

```shell
sudo apt update
sudo apt install ros-humble-hardware-interface ros-humble-ros-gz-interfaces ros-humble-moveit-visual-tools ros-humble-controller-manager ros-humble-ros-gzharmonic ros-humble-joint-trajectory-controller
``` 

> [!IMPORTANT]  
> If you have already installed `gz-sim-yarp-plugins` on your machine when reaching this section of the guide, it is likely that a specific `gz-sim` release was also installed through apt as a dependency. To avoid compatibility issues, please ensure that the Gazebo version you've already installed matches the ros2 ones (in this case, `Gazebo harmonic` is preferred).

## Install

To install the project from source, please follow **only one** of the following two sections.

### Install with colcon

In your ROS 2 workspace, clone the repo:

```shell
cd ~/<ros2_ws>/src
git clone https://github.com/icub-tech-iit/xcub-moveit2
```

and then build the environment:

```shell
cd ..
source /opt/ros/humble/setup.bash 
colcon build
source install/setup.bash
```

### Install within the robotology-superbuild

You may want to install `xcub-moveit2` via the robotology-superbuild. To get its when using the robotology-superbuild, please enable the `ROBOTOLOGY_USES_MOVEIT` CMake option of the superbuild.

### Install as a single CMake project

If you want to build this repository as a single CMake project, you can use the `CMakeLists.txt` provided in `xcub_moveit_all_packages`:

```shell
git clone https://github.com/icub-tech-iit/xcub-moveit2/
cd xcub-moveit2/xcub_moveit_all_packages
cmake -Bbuild -S. -DCMAKE_INSTALL_PREFIX=<install_prefix>
cmake --build build
cmake --install build

# Make ROS configuration files available in [ament index](https://github.com/ament/ament_index)
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:<install_prefix>
```

## Usage on real hardware

If you want to control a yarp-based robot (like iCub) using the proposed framework, you have to make sure that your machine is in communication with the robot one on which ros2 is installed. For this reason, be sure that your laptop and the robot machine are on the same network:

- configure your [yarpserver](https://www.yarp.it//v3.5/yarp.html#yarp_conf) as to be the one running on the robot environment;
- after checking your ip address and the robot one, customize the `cyclonedds.xml` file in this repository as follow:
  
![cyclonedds](assets/cyclonedds.jpeg)
- export this configuration file in the `.bashrc` in order to be used by `Cyclone DDS`:

```
export CYCLONEDDS_URI=/<path/to/your/ros2_ws>/src/xcub-moveit2/cyclonedds.xml
```
- repeat the last two steps on the robot machine, changing the ip address accordingly.


However, if you want to work in simulation instead of on the real hardware, you can skip the list above.

## Package description

This section aims to give a brief description of what each package contains.

### xcub_ros2_controllers

This package contains `xcub_ros2_controllers` plugin that is used in [ros2_control](https://control.ros.org/humble/index.html) framework. It includes:

- a `position state interface` used to read the position of each joint;
- a `velocity state interface` used to read the velocity of each joint;
- a `position command interface` used to forward the desired position to the joints.

### xcub_moveit_robot

This package contains some launch files, depending on the nodes you want to run.

- **`robot.launch.py`**: it allows to start up a set of nodes to bring up the robot. It spawns the model on `rviz`, publishes the state of the robot to `tf2` topic and starts the [move_group](https://moveit.picknik.ai/main/doc/concepts/move_group.html) node, which provides some ROS actions and services for using MoveIt 2 with your robot.
- **`robot_sim.launch.py`**: it's the same as before, but it spawns the model also in `gazebo` environment to work with the simulated robot.
- **`robot_controls.launch.py`**: this launch file allows to run the `controller manager` node for ros2_control and the nodes for the single controllers (one for each part).
- **`circle_demo.launch.py`** and **`grasp_demo.launch.py`**: as the name suggests, they are two examples of commanding the robot in the cartesian space using `torso + right_arm` as planning group.

Before running the nodes, make sure that your environment variable `YARP_ROBOT_NAME` is properly set with the name of your robot. If not, set it for each shell according to the chosen model, for example:

```shell
export YARP_ROBOT_NAME="iCubGenova11"
```

or, for simulated models:

```shell
export YARP_ROBOT_NAME="iCubGazeboV2_5"
```

### icub_moveit_config

This package contains the configuration files to make iCub working with MoveIt2. In particular, each of the parts of iCub robot (head, left_arm, right_arm, torso, left_leg and right_leg) are defined in terms of `planning group`, and for each of them a ros2_control of type `FollowJointTrajectory` is set.

### ergocub_moveit_config

It contains the same information described in the previous paragraph, but customized with ergoCub specs.

### xcub_moveit_test_controller

In this folder, a test to sample the reaching space is available. It can be run as a ros2 node with the provided launch file called `test_controller.launch.py`. For this purpose, you can find more info in the [Use case](#use-case) paragraph above.

Moreover, a detailed report about the controller performance can be found [here](https://github.com/icub-tech-iit/xcub-moveit2/blob/master/xcub_test_controller/README.md#test-on-the-controller-performance) 📝.

## Use case

### Run the demos

As described in the previous section, this repository contains two demos that show the possibility to control a yarp-based robot within the MoveIt2 framework in the Cartesian space. First of all, make sure that `yarpserver` is running on your machine. Then, you can start launching a basic simulation:

```shell
# Build the packages within your ros2 workspace
cd ~/<ros2_ws>
source /opt/ros/humble/setup.bash 
colcon build
source install/setup.bash

# Remember to check if your robot name variable is set. If not:
export YARP_ROBOT_NAME="iCubGazeboV2_5"

# Launch the start-up nodes
ros2 launch xcub_moveit_robot robot_sim.launch.py
```

In this way, both rviz2 and gz-sim windows are opened with the iCub model spawned in the two environments. At this point, open another shell, build and source the enviroment and then launch the ros2_control nodes:

```shell
cd ~/<ros2_ws>
source /opt/ros/humble/setup.bash 
colcon build
source install/setup.bash
export YARP_ROBOT_NAME="iCubGazeboV2_5"

# Launch the ros2_control nodes
ros2 launch xcub_moveit_robot robot_controls.launch.py
```

To verify that everthing has been done successfully, you can run in a separate shell:

```
ros2 control list_controllers
```
Here, you should see all the controllers loaded and their state (i.e. active or inactive).

Finally, open another shell and run the chosen demo, for example:

```shell
cd ~/<ros2_ws>
source /opt/ros/humble/setup.bash 
colcon build
source install/setup.bash
export YARP_ROBOT_NAME="iCubGazeboV2_5"

# For the grasping demo
ros2 launch xcub_moveit_robot grasp_demo.launch.py
```

If you want to see iCub performing a circle movement, instead of the last line, you can run:

```shell
ros2 launch xcub_moveit_robot circle_demo.launch.py
```

and follow the instructions on the third shell you opened.
You should have something like this:

<video src="assets/circle.mp4" controls title="Demos"></video>


### Controller performance

If you want to test the ros2_control `xcub_ros2_controllers` performance, you can rely to the `xcub_moveit_test_controller` package. It contains a simple script to sample the reaching space in front of iCub model. To run the simulation, please follow the first two steps described in the [Run the demos](#run-the-demos) section. After that, open another shell, source your workspace and then launch:

```shell
ros2 launch xcub_moveit_test_controller test_controller.launch.py
```

In this way, the test will start and the acquired data in terms of ideal and real poses are saved in two separated files in your current directory. Finally, you can plot them using the Matlab scripts provided in the `utils` folder.
