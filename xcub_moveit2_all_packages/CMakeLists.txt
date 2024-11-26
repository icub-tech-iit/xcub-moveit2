cmake_minimum_required(VERSION 3.22)
project(xcub-moveit2)

# This file is used to easily build the whole xcub-moveit2 repo
# as a single CMake project. To avoid interfering with the existing
# colcon workflows, this CMakeLists.txt is contained in a subfolder
# of the repo called all_packages, that contains a COLCON_IGNORE
# so that it will be excluded automatically by the colcon build command

# The order of inclusion was taken from colcon graph output
add_subdirectory(../ergocub_moveit_config ${CMAKE_CURRENT_BINARY_DIR}/ergocub_moveit_config)
add_subdirectory(../grasp_moveit ${CMAKE_CURRENT_BINARY_DIR}/grasp_moveit)
add_subdirectory(../icub_moveit_config ${CMAKE_CURRENT_BINARY_DIR}/icub_moveit_config)
add_subdirectory(../robot_controller ${CMAKE_CURRENT_BINARY_DIR}/robot_controller)
add_subdirectory(../robot_moveit ${CMAKE_CURRENT_BINARY_DIR}/robot_moveit)
add_subdirectory(../test_controller ${CMAKE_CURRENT_BINARY_DIR}/test_controller)

