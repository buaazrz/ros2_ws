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

    std::vector<double> pose_goal = {0.5500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100}; 
    if (!task_utils::execute_motion(node, "CartesianMotionController/goal", pose_goal)) {
        rclcpp::shutdown(); return 1;
    }

    std::this_thread::sleep_for(1s);

    if (!task_utils::activate_controller(node, controller_client, "CartesianTrajectoryDianaController")) return 1;

    std::vector<double> init_traj_goal = {
        10, 0.5500, -0.1800, 0.2600, 0.0000, 0.0000, -0.3100
    };

    if (task_utils::execute_motion(node, "CartesianTrajectoryDianaController/goal", init_traj_goal)) {
        RCLCPP_INFO(node->get_logger(), "碰到了！力检测已触发，机械臂已停止。");
    }

    RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Cartesian Trajectory ===");
    
    if (!task_utils::activate_controller(node, controller_client, "CartesianTrajectoryController")) {
        rclcpp::shutdown(); return 1;
    }

    std::vector<double> traj_goal = {
        0,  0.5500, -0.1800, 0.2600, 0.0000, -0.0000, -0.3100, 
        5,  0.5500, -0.0000, 0.2600, 0.0000, -0.0000, -0.3100,  
        10, 0.5500,  0.1800, 0.2600, 0.0000, -0.0000, -0.3100,  
        15, 0.4500,  0.1800, 0.2600, 0.0000, -0.0000, -0.3100,  
        20, 0.4500, -0.0000, 0.2600, 0.0000, -0.0000, -0.3100,  
        25, 0.4500, -0.1800, 0.2600, 0.0000, -0.0000, -0.3100, 
        30, 0.5500, -0.1800, 0.2600, 0.0000, -0.0000, -0.3100
    };
    
    if (!task_utils::execute_motion(node, "CartesianTrajectoryController/goal", traj_goal)) {
        rclcpp::shutdown(); return 1;
    }

    // =========================================================
    // 阶段 3: 任务结束，失活所有已调用的控制器
    // =========================================================
    RCLCPP_INFO(node->get_logger(), "=== PHASE 3: Deactivating Controllers ===");

    task_utils::deactivate_controller(node, controller_client, "CartesianTrajectoryController");
    task_utils::deactivate_controller(node, controller_client, "CartesianMotionController");

    RCLCPP_INFO(node->get_logger(), "=== ALL TASKS COMPLETED SUCCESSFULLY ===");
    rclcpp::shutdown();
    return 0;
}