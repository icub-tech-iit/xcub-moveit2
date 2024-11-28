cmake_minimum_required(VERSION 3.22)
project(xcub-moveit2)

# This file is used to easily build the whole xcub-moveit2 repo
# as a single CMake project. To avoid interfering with the existing
# colcon workflows, this CMakeLists.txt is contained in a subfolder
# of the repo called all_packages, that contains a COLCON_IGNORE
# so that it will be excluded automatically by the colcon build command

# The order of inclusion was taken from colcon graph output
add_subdirectory(../xcub_ergocub_moveit_config ${CMAKE_CURRENT_BINARY_DIR}/xcub_ergocub_moveit_config)
add_subdirectory(../xcub_moveit_grasp ${CMAKE_CURRENT_BINARY_DIR}/xcub_moveit_grasp)
add_subdirectory(../xcub_icub_moveit_config ${CMAKE_CURRENT_BINARY_DIR}/xcub_icub_moveit_config)
add_subdirectory(../xcub_ros2_controllers ${CMAKE_CURRENT_BINARY_DIR}/xcub_ros2_controllers)
add_subdirectory(../xcub_moveit_robot ${CMAKE_CURRENT_BINARY_DIR}/xcub_moveit_robot)
add_subdirectory(../xcub_moveit_test_controller ${CMAKE_CURRENT_BINARY_DIR}/xcub_moveit_test_controller)

