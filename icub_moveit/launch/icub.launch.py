import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions.execute_process import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None
    
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
    
def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
    )

    moveit_config = (
        MoveItConfigsBuilder("icub")
        .robot_description(file_path="config/iCub.urdf.xacro")
        .to_moveit_configs()
    )
    model_spawner = Node(package='icub_moveit', executable='spawn_icub.py', name='spawn_entity', output='screen', 
            parameters=[moveit_config.robot_description]
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
        parameters=[moveit_config.robot_description,
                    {"use_sim_time": True}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("icub_moveit_config"),
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

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
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
        arguments=["torso_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    robot_description_semantic_file = load_file("icub_moveit_config", "config/iCub.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_file}

    kinematic_yaml_file = load_yaml("icub_moveit_config", "config/kinematics.yaml")
    robot_description_kinematic = {"robot_description_kinematic": kinematic_yaml_file}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_file = load_yaml("icub_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_file)

    moveit_controllers_file = load_yaml("icub_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {"moveit_simple_controller_manager": moveit_controllers_file,
                          "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"}
    
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.robot_description,
                    robot_description_semantic,
                    robot_description_kinematic,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                    trajectory_execution,
                    planning_scene_monitor_parameters,
                    {"use_sim_time": True}],
    )

    rviz_moveit = os.path.join(get_package_share_directory("icub_moveit_config"), "config/moveit.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_moveit],
        parameters=[moveit_config.robot_description, 
                    robot_description_semantic,
                    robot_description_kinematic,
                    ompl_planning_pipeline_config,
                    {"use_sim_time":True}]
    )

    return LaunchDescription([
            gazebo,
            model_spawner,
            static_tf,
            robot_state_publisher,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=model_spawner,
                    on_exit=[ros2_control_node, left_arm_controller_spawner, right_arm_controller_spawner, left_leg_controller_spawner, right_leg_controller_spawner, head_controller_spawner, torso_controller_spawner]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=left_arm_controller_spawner,
                    on_exit=[
                        TimerAction(
                            period=1.0,
                            actions=[rviz2, move_group_node]
                        )
                    ]
                )
            )
        ]
    )