import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
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

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("icub_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    l_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_arm_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        robot_state_publisher,
        static_tf,
        ros2_control_node,
        l_arm_controller_spawner,

        ExecuteProcess(
            cmd=['ros2', 'launch', 'icub_moveit_config', 'moveit_rviz.launch.py'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'], 
            output='screen'
            ),

        Node(package='icub_moveit', executable='spawn_icub.py', name='spawn_entity', output='screen', 
             parameters=[moveit_config.to_dict()]
            )
    ])

