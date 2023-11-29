#include <cmath>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

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

  // Cartesian path
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  bool avoid_collisions = false;
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  // Grasping (from side)
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to reach the object");
  geometry_msgs::msg::Pose reach = start_pose;
  std::vector<geometry_msgs::msg::Pose> waypoints3;
  moveit_msgs::msg::RobotTrajectory reaching;

  // open
  reach.position.x = -0.25;
  reach.position.y = 0.298;
  reach.position.z = 0.090;
  waypoints3.clear();
  waypoints3.push_back(reach);
  move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, reaching, avoid_collisions);
  RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, reaching, avoid_collisions) * 100.0);

  visual_tools.deleteAllMarkers();  
  visual_tools.publishAxis(reach);
  visual_tools.trigger();
  move_group.execute(reaching);

  rclcpp::sleep_for(std::chrono::milliseconds(2s));

  // pre-grasp pose
  reach.position.x = -0.348;
  reach.position.y = 0.160;
  reach.position.z = -0.065;
  tf2::Quaternion pregrasp_orientation;
  pregrasp_orientation.setRotation({-0.082, 0.745, -0.662}, 3.003); //from axis-angle to quaternion
  reach.orientation = tf2::toMsg(pregrasp_orientation);
  waypoints3.clear();
  waypoints3.push_back(reach);
  move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, reaching, avoid_collisions);
  RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, reaching, avoid_collisions) * 100.0);

  visual_tools.deleteAllMarkers();  
  visual_tools.publishAxis(reach);
  visual_tools.trigger();
  move_group.execute(reaching);

  rclcpp::sleep_for(std::chrono::milliseconds(3s));

  // reach
  reach.position.x = -0.349;
  reach.position.y = 0.078;
  reach.position.z = -0.091;
  tf2::Quaternion grasp_orientation({-0.152, 0.702, -0.696}, 2.804); //from axis-angle to quaternion
  reach.orientation = tf2::toMsg(grasp_orientation);
  waypoints3.clear();
  waypoints3.push_back(reach);
  move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, reaching, avoid_collisions);
  RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, reaching, avoid_collisions) * 100.0);

  visual_tools.deleteAllMarkers();  
  visual_tools.publishAxis(reach);
  visual_tools.trigger();
  move_group.execute(reaching);

  // Home
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to home");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // move_group.setJointValueTarget(joint_values); 
  // move_group.move();

  waypoints3.clear();
  rclcpp::sleep_for(std::chrono::milliseconds(1s));

  waypoints3.push_back(start_pose);
  moveit_msgs::msg::RobotTrajectory inv_trajectory;
  double fraction = move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, inv_trajectory, avoid_collisions);
  RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", fraction * 100.0);
  move_group.execute(inv_trajectory);
  
  rclcpp::shutdown();
  return 0;
}