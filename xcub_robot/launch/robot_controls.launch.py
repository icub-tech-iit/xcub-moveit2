import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

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

    moveit_config = MoveItConfigsBuilder("xcub_"+robot_name).to_moveit_configs()

    ros2_controllers_path = os.path.join(
        get_package_share_directory("xcub_"+robot_name+"_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    right_arm_torso_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_torso_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": True}],
    )

    left_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_leg_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    right_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_leg_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    torso_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["torso_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        ros2_control_node,
        right_arm_torso_controller_spawner,
        right_arm_controller_spawner,
        left_arm_controller_spawner,
        head_controller_spawner,
        torso_controller_spawner,
    ])