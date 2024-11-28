import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource

def check_robot_name():
    try:
        if "icub" in os.environ["YARP_ROBOT_NAME"].casefold():
            yarp_robot_name = "icub"
        elif "ergocub" in os.environ["YARP_ROBOT_NAME"].casefold():
            yarp_robot_name = "ergocub"
        return yarp_robot_name
    except Exception as e:
        print(f"Caught exception: env variable {e} is not set, please provide it.")
    
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

    robot_name = check_robot_name()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
    )

    config = (
        MoveItConfigsBuilder("xcub_"+robot_name)
        .robot_description(file_path="config/"+robot_name+".urdf.xacro")
        .robot_description_semantic(file_path="config/"+robot_name+".srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    model_spawner = Node(
        package='xcub_robot', 
        executable='spawn_model.py', 
        name='spawn_entity', 
        output='screen', 
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[config.robot_description,
                    {"use_sim_time": True}],
    )

    robot_description_semantic_file = load_file("xcub_"+robot_name+"_moveit_config", "config/"+robot_name+".srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_file}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_file = load_yaml("xcub_"+robot_name+"_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_file)

    moveit_controllers_file = load_yaml("xcub_"+robot_name+"_moveit_config", "config/moveit_controllers.yaml")
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

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config.robot_description,
                    robot_description_semantic,
                    config.robot_description_kinematics,
                    ompl_planning_pipeline_config,
                    moveit_controllers,
                    trajectory_execution,
                    planning_scene_monitor_parameters,
                    move_group_capabilities,
                    {"use_sim_time": True}],
    )

    rviz_moveit = os.path.join(get_package_share_directory("xcub_"+robot_name+"_moveit_config"), "config/moveit.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_moveit],
        parameters=[config.robot_description, 
                    robot_description_semantic,
                    config.robot_description_kinematics,
                    config.planning_pipelines,
                    ompl_planning_pipeline_config,
                    planning_scene_monitor_parameters,
                    {"use_sim_time": True}]
    )

    return LaunchDescription([
            gazebo,
            model_spawner,
            robot_state_publisher,
            rviz2,
            move_group_node
        ]
    )