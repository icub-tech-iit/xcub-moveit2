import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def check_robot_name():
    try:
        if "icub" in os.environ["YARP_ROBOT_NAME"].casefold():
            yarp_robot_name = "icub"
        elif "ergocub" in os.environ["YARP_ROBOT_NAME"].casefold():
            yarp_robot_name = "ergocub"
        return yarp_robot_name
    except Exception as e:
        print(f"Caught exception: env variable {e} is not set, please provide it.")

def generate_launch_description():

    robot_name = check_robot_name()

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