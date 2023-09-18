import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

robot_name = os.environ["YARP_ROBOT_NAME"]

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(robot_name).to_moveit_configs()

    circle_demo = Node(
        name="robot_moveit",
        package="robot_moveit",
        executable="robot_moveit",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        circle_demo
    ])