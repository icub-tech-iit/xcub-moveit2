#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "icub_controller/icub_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;
namespace icub_controller 
{
  ICubController::ICubController()
  {
  }
  ICubController::~ICubController()
  {
  }

  bool ICubController::getJoints(const std::vector<hardware_interface::ComponentInfo> & joints)
  {
    std::vector<std::string> allJoints;
    auto namesRequest = std::make_shared<yarp_control_msgs::srv::GetJointsNames::Request>();
    while(!getJointsNamesClient->wait_for_service(1s)) 
    {
      if(!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "Request not available.");
        return false;
      }
      
      RCLCPP_INFO(node->get_logger(), "service getJointsNamesClient not available, waiting again...");
    }

    auto namesFuture = getJointsNamesClient->async_send_request(namesRequest);
    if(rclcpp::spin_until_future_complete(node, namesFuture) == rclcpp::FutureReturnCode::SUCCESS) 
    {
      auto namesRespond = namesFuture.get();
      RCLCPP_INFO(node->get_logger(), "Got joints names");
      allJoints = namesRespond.get()->names;
    }

    else 
    {
      RCLCPP_ERROR(node->get_logger(),"Failed to get joints names");
      return false;
    }

    for(const auto& joint : joints)
    {
      jointNames.push_back(joint.name);
      RCLCPP_INFO(node->get_logger(), "%s", joint.name.c_str()); 
    }
    
    auto modeRequest = std::make_shared<yarp_control_msgs::srv::GetControlModes::Request>();
    while(!getControlModesClient->wait_for_service(1s))
    {
      if(!rclcpp::ok())
      {
        RCLCPP_ERROR(node->get_logger(), "modeRequest not available");
        return false;
      }
      
      RCLCPP_INFO(node->get_logger(), "service getControlModeClient not available, waiting again...");
    }

    modeRequest->names = jointNames;

    auto modeFuture = getControlModesClient->async_send_request(modeRequest);
    if(rclcpp::spin_until_future_complete(node, modeFuture) == rclcpp::FutureReturnCode::SUCCESS) 
    {
      auto modeResponse = modeFuture.get();

      for(size_t i = 0; i < jointNames.size(); i++)
      {
        if(modeResponse->modes[i] != "POSITION")
        {
          RCLCPP_ERROR(node->get_logger(), "Joint %s is not in control mode position", jointNames[i].c_str());
          return false;
        }
          
        RCLCPP_INFO(node->get_logger(), "Joint %s is in %s control mode", jointNames[i].c_str(), modeResponse->modes[i].c_str());
      }
    }

    else 
    {
      RCLCPP_ERROR(node->get_logger(),"Failed to get control modes");
      return false;
    }

  return true;

  }

  hardware_interface::CallbackReturn ICubController::getCurrentValues(const std::vector<hardware_interface::ComponentInfo>& joints)
  {

    if(!getJoints(joints))
    {
      return CallbackReturn::ERROR;
    }

    hwPosition.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    newPosition.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    hwVelocity.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());

    size_t i = 0;

    for(const auto& joint : joints)
    {
      newPosition[i] = 0.0;
      hwVelocity[i] = 0.0;
      hwPosition[i] = 0.0;
    }

    auto posRequest = std::make_shared<yarp_control_msgs::srv::GetPosition::Request>();
    posRequest->names = jointNames;

    RCLCPP_INFO(node->get_logger(), "Joint name size is: %d", jointNames.size());

    while (!getPositionClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(node->get_logger(), "service getPositionClient not available, waiting again...");
    }
    
    auto posFuture = getPositionClient->async_send_request(posRequest);
    auto posResponse = std::make_shared<yarp_control_msgs::srv::GetPosition::Response>();

    if(rclcpp::spin_until_future_complete(node, posFuture) == rclcpp::FutureReturnCode::SUCCESS) 
    {
      RCLCPP_INFO(node->get_logger(), "Got joints positions");
      posResponse = posFuture.get();
    }

    else 
    {
      RCLCPP_ERROR(node->get_logger(),"Failed to get joints positions");
      return CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < jointNames.size(); i++)
    {
      newPosition[i] = posResponse->positions[i];
      hwPosition[i] = posResponse->positions[i];

      RCLCPP_INFO(node->get_logger(), "hwPosition[%f]", hwPosition[i]);
      RCLCPP_INFO(node->get_logger(), "newPosition[%f]", newPosition[i]);
    }

    return CallbackReturn::SUCCESS;
  }
  
  hardware_interface::CallbackReturn ICubController::on_init(const hardware_interface::HardwareInfo & info)
  {    
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    nodeName = info_.hardware_parameters["node_name"];
    msgsName = info_.hardware_parameters["msgs_name"];
    
    positionTopicName = msgsName + "/position";
    getJointsNamesClientName = msgsName + "/get_joints_names";
    getPositionClientName = msgsName + "/get_position";
    getControlModesClientName = msgsName + "/get_modes";
        
    node = rclcpp::Node::make_shared(nodeName);
    
    positionPublisher = node->create_publisher<yarp_control_msgs::msg::Position>(positionTopicName, 10);
    if(!positionPublisher)
    {
      RCLCPP_ERROR(node->get_logger(),"Could not initialize the position publisher");
      return CallbackReturn::ERROR;
    }
    else
      RCLCPP_INFO(node->get_logger(), "Position publisher initializated");

    getJointsNamesClient = node->create_client<yarp_control_msgs::srv::GetJointsNames>(getJointsNamesClientName);
    if(!getJointsNamesClient) 
    {
      RCLCPP_ERROR(node->get_logger(), "getJointsNamesClient not available.");
      return CallbackReturn::ERROR;
    }
    else
      RCLCPP_INFO(node->get_logger(), "getJointsNames client initializated");

    getPositionClient = node->create_client<yarp_control_msgs::srv::GetPosition>(getPositionClientName);
    if(!getPositionClient) 
    {
      RCLCPP_ERROR(node->get_logger(), "getPositionClient not available.");
      return CallbackReturn::ERROR;
    }
    else
      RCLCPP_INFO(node->get_logger(), "getPosition client initializated");

    getControlModesClient = node->create_client<yarp_control_msgs::srv::GetControlModes>(getControlModesClientName);
    if(!getControlModesClient)
    {
      RCLCPP_ERROR(node->get_logger(), "getControlModes client not available.");
      return CallbackReturn::ERROR;
    }
    else
      RCLCPP_INFO(node->get_logger(), "getControlModes client initializated");

    return getCurrentValues(info_.joints);
  }  

  std::vector<hardware_interface::StateInterface> ICubController::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> stateInterfaces;
    for(size_t i = 0; i < info_.joints.size(); i++)
    {
      stateInterfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hwPosition[i]));
      
      stateInterfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hwVelocity[i]));
    }
    
    return stateInterfaces;
  }

  std::vector<hardware_interface::CommandInterface> ICubController::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> cmdInterfaces;
    for(size_t i = 0; i < info_.joints.size(); i++)
    {
      cmdInterfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &newPosition[i]));
    }

    return cmdInterfaces;
  }

  hardware_interface::CallbackReturn ICubController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (size_t i = 0; i < hwPosition.size(); i++)
    {
      hwPosition[i] = 0;
      newPosition[i] = 0;
      hwVelocity[i] = 0;
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ICubController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ICubController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ICubController::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ICubController::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if(newPosition == hwPosition)
    {
      return hardware_interface::return_type::OK;
    }

    yarp_control_msgs::msg::Position positionToSend;
    positionToSend.names = jointNames;
    positionToSend.positions = newPosition;

    positionPublisher->publish(positionToSend);
    hwPosition = newPosition;

    return hardware_interface::return_type::OK;
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(icub_controller::ICubController, hardware_interface::SystemInterface)

