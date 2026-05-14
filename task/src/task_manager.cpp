#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>

// 引入我们自己写的辅助函数头文件
#include "task/task_helpers.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task_manager_node");

    auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>(
        "control_node/control_command");
        
    if (!controller_client->wait_for_service(2s)) {
        RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Cartesian Motion ===");
    
    if (!task_utils::activate_controller(node, controller_client, "CartesianMotionController")) {
        rclcpp::shutdown(); return 1;
    }

    std::vector<double> pose_goal = {0.536320, -0.163060, 0.356040, 0.003903, 0.022033, -0.164466}; 
    if (!task_utils::execute_motion(node, "CartesianMotionController/goal", pose_goal)) {
        rclcpp::shutdown(); return 1;
    }

    if (!task_utils::activate_controller(node, controller_client, "CartesianTrajectoryDianaController")) return 1;
    
    std::vector<double> init_traj_goal = {
        10, 0.536320, -0.163060, 0.306040, 0.003903, 0.022033, -0.164466,
    };

    std::this_thread::sleep_for(5s);
    
    if (task_utils::execute_motion(node, "CartesianTrajectoryDianaController/goal", init_traj_goal)) {
        RCLCPP_INFO(node->get_logger(), "碰到了！力检测已触发，机械臂已停止。");
    }

    
    // CartesianTrajectoryDianaController VariableImpedanceController
    if (!task_utils::activate_controller(node, controller_client, "VariableImpedanceController")) {
        rclcpp::shutdown(); return 1;
    }
    
    std::this_thread::sleep_for(5s);
    std::vector<double> traj_goal = {
        5,  0.536320, -0.163060, 0.315040, 0.003903, 0.022033, -0.164466, 
        15, 0.536320, -0.0500, 0.315040,0.003903, 0.022033, -0.164466,  
        25, 0.536320, -0.163060, 0.315040, 0.003903, 0.022033, -0.164466,  
    };

    
    if (!task_utils::execute_motion(node, "VariableImpedanceController/goal", traj_goal)) {
        rclcpp::shutdown(); return 1;
    }

    task_utils::deactivate_controller(node, controller_client, "VariableImpedanceController"); 

    RCLCPP_INFO(node->get_logger(), "=== ALL TASKS COMPLETED SUCCESSFULLY ===");
    rclcpp::shutdown();
    return 0;
}