#include <cmath>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <random>

#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("xcub_test_controller");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "right_arm_torso";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::core::RobotState start_state(*move_group.getCurrentState());

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  std::vector<double> joint_values;
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  start_state.copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "root_link", "display_contacts",
                                                      move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  // define cartesian trajectories
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  bool avoid_collisions = false;
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  int count = 0;
  int rep = 0;

  std::ofstream data("poses_ws_1e-4_distance_dw.txt");
  std::ofstream current_data("current_data_1e-4_distance_dw.txt");

  std::vector<std::vector<double>> data_poses;
  std::vector<std::vector<double>> current_poses;
    
  std::vector<double> ws_x_limits(2, 0.0);
  std::vector<double> ws_y_limits(2, 0.0);
  std::vector<double> ws_z_limits(2, 0.0);

  ws_x_limits[0] = -0.3;
  ws_x_limits[1] = -0.1;
  ws_y_limits[0] = -0.1;
  ws_y_limits[1] = 0.3;
  ws_z_limits[0] = -0.2;
  ws_z_limits[1] = 0.2;

  double dist = 0.05;
  std::vector<double> set_pose(9, 0.0); 
  std::vector<double> real_pose(7, 0.0);

  for (auto i = ws_x_limits[0]; i <= ws_x_limits[1]; i = i + dist)
  {
    for (auto j = ws_y_limits[0]; j <= ws_y_limits[1]; j = j + dist)
    {
      for (auto k = ws_z_limits[0]; k <= ws_z_limits[1]; k = k + dist)
      {
        rep++;

        set_pose[0] = i;
        set_pose[1] = j;
        set_pose[2] = k;

        geometry_msgs::msg::Pose rand_position;
        std::vector<geometry_msgs::msg::Pose> waypoints;

        rand_position.position.x = i;
        rand_position.position.y = j;
        rand_position.position.z = k;

        tf2::Quaternion orientation_hand; //downward
        orientation_hand.setRPY(0, -M_PI, 0); 
        rand_position.orientation = tf2::toMsg(orientation_hand);

        // tf2::Quaternion grasp_orientation({-0.152, 0.702, -0.696}, 2.804); //leftward
        // rand_position.orientation = tf2::toMsg(grasp_orientation);

        set_pose[3] = rand_position.orientation.x;    
        set_pose[4] = rand_position.orientation.y;
        set_pose[5] = rand_position.orientation.z;
        set_pose[6] = rand_position.orientation.w;

        waypoints.push_back(rand_position);

        moveit_msgs::msg::RobotTrajectory rand_trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, rand_trajectory, avoid_collisions);
        RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", fraction * 100.0);
        visual_tools.publishTrajectoryLine(rand_trajectory, joint_model_group);
        visual_tools.trigger();
        move_group.execute(rand_trajectory);

        if (fraction == 1.0)
        {
          count++;
          set_pose[7] = 1; //success
        } 

        else
          set_pose[7] = 0; //failure

        set_pose[8] = fraction * 100.0;

        data_poses.push_back(set_pose);

        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
        std::cout << "Current end effector pose " << ": x=" << current_pose.pose.position.x << ", y=" << current_pose.pose.position.y << ", z=" << current_pose.pose.position.z << std::endl;
        std::cout << "Rep number: " << rep << std::endl;
        
        real_pose[0] = current_pose.pose.position.x;
        real_pose[1] = current_pose.pose.position.y;
        real_pose[2] = current_pose.pose.position.z;
        real_pose[3] = current_pose.pose.orientation.x;
        real_pose[4] = current_pose.pose.orientation.y;
        real_pose[5] = current_pose.pose.orientation.z;
        real_pose[6] = current_pose.pose.orientation.w;

        current_poses.push_back(real_pose);

        waypoints.clear();
        rclcpp::sleep_for(std::chrono::milliseconds(1s));
        waypoints.push_back(start_pose);
        
        // plan the inverse trajectory in cartesian space
        moveit_msgs::msg::RobotTrajectory inv_trajectory;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, inv_trajectory, avoid_collisions);
        RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", fraction * 100.0);
        move_group.execute(inv_trajectory);
        
        rclcpp::sleep_for(std::chrono::milliseconds(1s));
      }
    }
  }
  
  RCLCPP_INFO(LOGGER, "How many time the 100%% is achieved? %d", count);

  for (const auto&d : data_poses) {
    data << d[0] << " "
    << d[1] << " "
    << d[2] << " "
    << d[3] << " "
    << d[4] << " "
    << d[5] << " "
    << d[6] << " "
    << d[7] << " "
    << d[8] << std::endl;
  }

  for (const auto&d : current_poses) {
    current_data << d[0] << " "
    << d[1] << " "
    << d[2] << " "
    << d[3] << " "
    << d[4] << " "
    << d[5] << " "
    << d[6] << std::endl;
  }

  data.close();
  current_data.close();

  rclcpp::shutdown();
  return 0;
}