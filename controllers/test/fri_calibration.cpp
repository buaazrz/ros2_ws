#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_control_msgs/action/robot_motion.hpp"
#include "robot_control_msgs/srv/control_command.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using ACTION = robot_control_msgs::action::RobotMotion;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;

// 辅助函数：发送目标并阻塞等待运动完成
bool move_to_target(rclcpp::Node::SharedPtr node, 
                    rclcpp_action::Client<ACTION>::SharedPtr action_client, 
                    const std::vector<double>& target_q)
{
    if (!action_client->wait_for_action_server(1s)) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available!");
        return false;
    }

    auto goal_msg = ACTION::Goal();
    goal_msg.target_position.data = target_q;

    auto send_goal_options = rclcpp_action::Client<ACTION>::SendGoalOptions();
    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Send goal failed");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return false;
    }

    auto result_future = action_client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Get result failed");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "Reached target successfully!");
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
        return false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("friction_identification_client");

    auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>("control_node/control_command");
    auto action_client = rclcpp_action::create_client<ACTION>(node, "JointMotionController/goal");

    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "JointMotionController");

    // ==========================================
    // 1. 激活控制器
    // ==========================================
    if(!controller_client->wait_for_service(1s)) {
        RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    auto request = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
    request->cmd_name = "activate";
    request->cmd_params = "JointMotionController";
    auto future = controller_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to active JointMotionController");
        rclcpp::shutdown(); return 1;
    }

    request->cmd_name = "add";
    request->cmd_params = "RobotStateBroadcaster";
    auto future2 = controller_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future2) != rclcpp::FutureReturnCode::SUCCESS || !future2.get()->result) {
        RCLCPP_ERROR(node->get_logger(), "Failed to active RobotStateBroadcaster");
        rclcpp::shutdown(); return 1;
    }

    // ==========================================
    // 2. 定义往复运动的起点A和终点B
    // ==========================================
    // 以辨识关节 1 (索引0) 为例。注意：运行前务必确保机械臂在这个角度内不会发生干涉！
    std::vector<double> point_A = { 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, -0.6};
    std::vector<double> point_B = { 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.6};  
    
    // 初始先移动到点 A 准备
    if (param_client->wait_for_service(2s)) {
        param_client->set_parameters({rclcpp::Parameter("speed", 0.5)});
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500)); 
    RCLCPP_INFO(node->get_logger(), "Moving to initial start point A (Init speed: 0.5)...");
    move_to_target(node, action_client, point_A);  
    rclcpp::sleep_for(2s); // 停顿等待稳定
    if (rclcpp::ok())
    {
        // ==========================================
        // 3. 循环递增速度进行摩擦力辨识 (从 0.01 增加到 0.7，每次 +0.01)
        // ==========================================
        for (double current_speed = 0.02; current_speed <= 0.705; current_speed += 0.02)
        {
            RCLCPP_INFO(node->get_logger(), "==========================================");
            RCLCPP_INFO(node->get_logger(), "Starting new cycle. Setting speed to: %.2f", current_speed);
            
            // 【修改】通过参数客户端异步设置参数
            param_client->set_parameters({rclcpp::Parameter("speed", current_speed)});
            rclcpp::sleep_for(std::chrono::milliseconds(200)); // 给控制器更新参数的时间

            // 正向运动 (Forward) A -> B
            RCLCPP_INFO(node->get_logger(), "Moving Forward (A -> B)...");
            move_to_target(node, action_client, point_B);
            
            RCLCPP_INFO(node->get_logger(), "Wait 1s...");
            rclcpp::sleep_for(1s);

            // 反向运动 (Backward) B -> A
            RCLCPP_INFO(node->get_logger(), "Moving Backward (B -> A)...");
            move_to_target(node, action_client, point_A);

            RCLCPP_INFO(node->get_logger(), "Wait 1s...");
            rclcpp::sleep_for(1s);
        }

        // ==========================================
        // 4. 辨识结束，保存数据
        // ==========================================
        RCLCPP_INFO(node->get_logger(), "All cycles completed! Sending command to deactivate controller and save data...");
        request->cmd_name = "deactivate"; 
        request->cmd_params = "JointMotionController";
        auto future_stop = controller_client->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node, future_stop) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Controller deactivated and data saved successfully!");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to deactivate controller or save data took too long.");
        }
    }

    rclcpp::shutdown();
    return 0;
}