import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("icub")
        .robot_description(file_path="config/iCub.urdf.xacro")
        .robot_description_semantic(file_path="config/iCub.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"]
    )

    return LaunchDescription([
            move_group_node
        ]
)