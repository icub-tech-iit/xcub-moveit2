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

  // Hand down
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the hand palm down");
  geometry_msgs::msg::Pose hand_down = start_pose;
  std::vector<geometry_msgs::msg::Pose> waypoints1;

  tf2::Quaternion orientation_hand;
  orientation_hand.setRPY(0, -M_PI, 0);
  hand_down.orientation = tf2::toMsg(orientation_hand);

  hand_down.position.x = -0.3;
  hand_down.position.y = 0.2;
  hand_down.position.z = 0.02;
  waypoints1.push_back(hand_down);
  moveit_msgs::msg::RobotTrajectory hand;
  double fraction1 = move_group.computeCartesianPath(waypoints1, eef_step, jump_threshold, hand, avoid_collisions);
  RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", fraction1 * 100.0);
  visual_tools.publishAxis(hand_down);
  visual_tools.trigger();
  move_group.execute(hand);

  // Circle
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan a circular trajectory");

  double radius = 0.08;
  geometry_msgs::msg::Pose circle = hand_down;
  std::vector<geometry_msgs::msg::Pose> waypoints2;

  for (auto angle = 0.0; angle <= 2*M_PI; angle+=M_PI/24)
  {
    circle.position.y = hand_down.position.y - radius*cos(angle);
    circle.position.z = hand_down.position.z + radius*sin(angle);
    waypoints2.push_back(circle);
  }

  moveit_msgs::msg::RobotTrajectory circular_trajectory;
  double fraction2 = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, circular_trajectory, avoid_collisions);
  RCLCPP_INFO(LOGGER, "Cartesian path: %.2f%% achieved", fraction2 * 100.0);

  visual_tools.deleteAllMarkers();
  visual_tools.publishAxis(circle);
  visual_tools.publishTrajectoryLine(circular_trajectory, joint_model_group);
  visual_tools.trigger();
  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.execute(circular_trajectory);

  // Home
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to home");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  move_group.setJointValueTarget(joint_values); 
  move_group.move();

  rclcpp::shutdown();
  return 0;
}