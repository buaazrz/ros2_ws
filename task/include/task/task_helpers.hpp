#ifndef TASK_HELPERS_HPP_
#define TASK_HELPERS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>
#include <string>

#include "robot_control_msgs/srv/control_command.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"

namespace task_utils
{
    // 激活指定的控制器
    bool activate_controller(
        rclcpp::Node::SharedPtr node, 
        rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedPtr srv_client, 
        const std::string& controller_name);

    // 失活指定的控制器
    bool deactivate_controller(
        rclcpp::Node::SharedPtr node, 
        rclcpp::Client<robot_control_msgs::srv::ControlCommand>::SharedPtr srv_client, 
        const std::string& controller_name);

    // 给控制器发送 Action 目标并等待完成
    bool execute_motion(
        rclcpp::Node::SharedPtr node,   
        const std::string& action_name, 
        const std::vector<double>& goal_data);

} // namespace task_utils

#endif // TASK_HELPERS_HPP_