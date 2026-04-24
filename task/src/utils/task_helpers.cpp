#include "task/task_helpers.hpp"
#include <chrono>

using namespace std::chrono_literals;
using ACTION = robot_control_msgs::action::RobotMotion;

namespace task_utils
{

bool activate_controller(rclcpp::Node::SharedPtr node, 
                         rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedPtr srv_client, 
                         const std::string& controller_name)
{
    RCLCPP_INFO(node->get_logger(), "Attempting to activate: %s", controller_name.c_str());
    
    if (!srv_client->wait_for_service(2s)) {
        RCLCPP_ERROR(node->get_logger(), "Control service not available!");
        return false;
    }

    auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
    request->cmd_name = "activate";
    request->cmd_params = controller_name;

    auto future = srv_client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node, future);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service for %s", controller_name.c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully activated: %s", controller_name.c_str());
    return true;
}

bool deactivate_controller(rclcpp::Node::SharedPtr node, 
                           rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedPtr srv_client, 
                           const std::string& controller_name)
{
    RCLCPP_INFO(node->get_logger(), "Attempting to deactivate: %s", controller_name.c_str());
    
    if (!srv_client->wait_for_service(2s)) {
        RCLCPP_ERROR(node->get_logger(), "Control service not available!");
        return false;
    }

    auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
    request->cmd_name = "deactivate"; 
    request->cmd_params = controller_name;

    auto future = srv_client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node, future);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service for %s", controller_name.c_str());
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully deactivated: %s", controller_name.c_str());
    return true;
}

bool execute_motion(rclcpp::Node::SharedPtr node,   
                    const std::string& action_name, 
                    const std::vector<double>& goal_data)
{
    RCLCPP_INFO(node->get_logger(), "Sending goal to action server: %s", action_name.c_str());
    auto action_client = rclcpp_action::create_client<ACTION>(node, action_name);

    if (!action_client->wait_for_action_server(3s)) {
        RCLCPP_ERROR(node->get_logger(), "Action server %s not available!", action_name.c_str());
        return false;
    }

    auto goal_msg = ACTION::Goal();
    goal_msg.target_position.data = goal_data;

    auto handle_future = action_client->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node, handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal to %s", action_name.c_str());
        return false;
    }

    auto handle = handle_future.get();
    if (handle == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by %s", action_name.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Goal accepted, waiting for execution to finish...");
    auto result_future = action_client->async_get_result(handle);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result from %s", action_name.c_str());
        return false;
    }

    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node->get_logger(), "Action execution failed on %s", action_name.c_str());
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Motion executed successfully!");
    return true;
}

} // namespace task_utils