FROM ghcr.io/icub-tech-iit/docker-deployment-images/cd_superbuild-ros2:master-unstable_sources

WORKDIR /

# RUN apt update && apt install -y \
#     ros-humble-gazebo-msgs \
#     ros-humble-gazebo-ros \
#     ros-humble-rmw-cyclonedds-cpp \
#     ros-humble-ament-cmake-clang-format \
#     ros-humble-hardware-interface \
#     ros-humble-controller-manager \
#     ros-humble-moveit \
#     ros-humble-moveit-visual-tools \
#     ros-humble-moveit-ros-planning-interface

RUN apt update && apt install -y \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-hardware-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-ros-planning-interface
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /usr/local/bin/setup_robotology_tdd.sh