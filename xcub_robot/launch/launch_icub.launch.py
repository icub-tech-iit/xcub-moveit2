import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessStart
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("icub")
        .robot_description(file_path="config/icub.urdf.xacro")
        .robot_description_semantic(file_path="config/icub.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'], 
        output='screen'
    )

    model_spawner = Node(package='icub_moveit', executable='spawn_icub.py', name='spawn_entity', output='screen', 
            parameters=[moveit_config.to_dict()]
    )

    rviz2 = ExecuteProcess(
        cmd=['ros2', 'launch', 'xcub_icub_config', 'moveit_rviz.launch.py'],
        output='screen'
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "l_wrist_yaw"],
        parameters=[{"use_sim_time": True}]
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("xcub_icub_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path,
                    {"use_sim_time": True}],
        output="log",
    )

    l_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["l_arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
        # prefix=['xterm -e gdb -ex run --args'],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gazebo,
        RegisterEventHandler(
            OnProcessStart(
                target_action=gazebo,
                on_start=[model_spawner]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=model_spawner,
                on_completion=[rviz2]
            )
        ),
        move_group_node,
        robot_state_publisher,
        static_tf,

        RegisterEventHandler(
            OnProcessStart(
                target_action=rviz2,
                on_start=[ros2_control_node]
            )
        ),
        
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[l_arm_controller_spawner]
            )
        )
    ])

