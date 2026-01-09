FROM ghcr.io/icub-tech-iit/docker-deployment-images/cd_superbuild-ros2:master-unstable_sources

WORKDIR /

RUN apt update && apt install -y \
    ros-humble-ros-gz-interfaces \
    ros-humble-ros-gz \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-cmake-clang-format \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-ros-planning-interface

RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /usr/local/bin/setup_robotology_tdd.sh