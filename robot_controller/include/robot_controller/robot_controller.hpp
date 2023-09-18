#ifndef ROBOT_CONTROLLER__ROBOT_CONTROLLER_HPP_
#define ROBOT_CONTROLLER__ROBOT_CONTROLLER_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "visibility_control.h"

// yarp msgs and srv
#include "yarp_control_msgs/srv/get_available_control_modes.hpp"
#include "yarp_control_msgs/srv/get_control_modes.hpp"
#include "yarp_control_msgs/srv/get_joints_names.hpp"
#include "yarp_control_msgs/srv/get_position.hpp"
#include "yarp_control_msgs/srv/set_control_modes.hpp"
#include "yarp_control_msgs/msg/position.hpp"
#include "yarp_control_msgs/msg/position_direct.hpp"
#include "yarp_control_msgs/msg/velocity.hpp"

namespace robot_controller {
    
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RobotController : public hardware_interface::SystemInterface
    {            
        private:
            rclcpp::Publisher<yarp_control_msgs::msg::Position>::SharedPtr              positionPublisher;
            rclcpp::Publisher<yarp_control_msgs::msg::PositionDirect>::SharedPtr        positionDirectPublisher;
            rclcpp::Client<yarp_control_msgs::srv::GetJointsNames>::SharedPtr           getJointsNamesClient;
            rclcpp::Client<yarp_control_msgs::srv::GetControlModes>::SharedPtr          getControlModesClient;
            rclcpp::Client<yarp_control_msgs::srv::GetPosition>::SharedPtr              getPositionClient;
            rclcpp::Client<yarp_control_msgs::srv::SetControlModes>::SharedPtr          setControlModesClient;
            // rclcpp::Client<yarp_control_msgs::srv::GetAvailableControlModes>::SharedPtr getAvailableControlModesClient;

            rclcpp::Node::SharedPtr node;

            std::string nodeName;
            std::string msgsName;

            std::string positionTopicName;
            std::string positionDirectTopicName;
            std::string getJointsNamesClientName;
            std::string getPositionClientName;
            std::string getControlModesClientName;
            std::string setControlModesClientName;
            // std::string getAvailableControlModesClientName;

            std::vector<double> newPosition; //position command interface
            std::vector<double> hwPosition; //position state interface
            std::vector<double> hwVelocity; //velocity state interface

            std::vector<std::string> jointNames;

            bool getJoints(const std::vector<hardware_interface::ComponentInfo>& joints);
            hardware_interface::CallbackReturn getCurrentValues(const std::vector<hardware_interface::ComponentInfo>& joints);
            bool setControlMode();

        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(RobotController)
            ROBOT_CONTROLLER_PUBLIC
            RobotController();
            ~RobotController();

            ROBOT_CONTROLLER_PUBLIC hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
            ROBOT_CONTROLLER_PUBLIC hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
            ROBOT_CONTROLLER_PUBLIC hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
            ROBOT_CONTROLLER_PUBLIC hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            ROBOT_CONTROLLER_PUBLIC hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            ROBOT_CONTROLLER_PUBLIC hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            ROBOT_CONTROLLER_PUBLIC std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            ROBOT_CONTROLLER_PUBLIC std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    };
}

#endif