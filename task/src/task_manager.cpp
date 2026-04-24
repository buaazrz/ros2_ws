#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <vector>
#include <string>

// 引入该框架自定义的服务和动作消息
#include "robot_control_msgs/srv/control_command.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"

using namespace std::chrono_literals;
using ACTION = robot_control_msgs::action::RobotMotion;

// =========================================================================
// 辅助函数 1：激活指定的控制器
// =========================================================================
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

// =========================================================================
// 辅助函数 2：失活指定的控制器
// =========================================================================
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
    // 这里将命令改为 deactivate
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

// =========================================================================
// 辅助函数 3：给控制器发送 Action 目标并等待完成
// =========================================================================
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

    // 发送目标并等待服务器接收
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

    // 等待动作执行完成
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

// =========================================================================
// 主逻辑：在这个任务链中按顺序调用不同的控制器
// =========================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task_manager_node");

    // 创建通用控制器切换的 Service Client
    auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>(
        "control_node/control_command");
    if (!controller_client->wait_for_service(1s))
    {
        RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    // ---------------------------------------------------------
    // 阶段 1: 使用 CartesianMotionController 到达初始准备位姿
    // ---------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Cartesian Motion ===");
    if (!activate_controller(node, controller_client, "CartesianMotionController")) {
        rclcpp::shutdown(); return 1;
    }

    // 给 CartesianMotionController 发送位姿 
    std::vector<double> pose_goal = {0.5500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100}; 
    if (!execute_motion(node, "CartesianMotionController/goal", pose_goal)) {
        rclcpp::shutdown(); return 1;
    }

    // 停顿一下，让机器人在切换控制器前保持稳定
    std::this_thread::sleep_for(1s);

    // ---------------------------------------------------------
    // 阶段 2: 切换为 CartesianTrajectoryController 执行轨迹
    // ---------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Cartesian Trajectory ===");
    
    // 激活 Trajectory Controller
    if (!activate_controller(node, controller_client, "CartesianTrajectoryController")) {
        rclcpp::shutdown(); return 1;
    }

    // 发送多点平铺（Flatten）轨迹数据：[Time, X, Y, Z, Rx, Ry, Rz]
    std::vector<double> traj_goal = {
        0,  0.5500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100, 
        5,  0.5500, -0.0000, 0.3600, 0.0000, -0.0000, -0.3100,  
        10, 0.5500,  0.1800, 0.3600, 0.0000, -0.0000, -0.3100,  
        15, 0.4500,  0.1800, 0.3600, 0.0000, -0.0000, -0.3100,  
        20, 0.4500, -0.0000, 0.3600, 0.0000, -0.0000, -0.3100,  
        25, 0.4500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100, 
        30, 0.5500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100
    };
    
    if (!execute_motion(node, "CartesianTrajectoryController/goal", traj_goal)) {
        rclcpp::shutdown(); return 1;
    }

    // ---------------------------------------------------------
    // 阶段 3: 任务结束，失活所有已调用的控制器
    // ---------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "=== PHASE 3: Deactivating Controllers ===");

    deactivate_controller(node, controller_client, "CartesianTrajectoryController");

    RCLCPP_INFO(node->get_logger(), "=== ALL TASKS COMPLETED SUCCESSFULLY ===");
    rclcpp::shutdown();
    return 0;
}