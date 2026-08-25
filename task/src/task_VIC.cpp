#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>
#include "task/task_helpers.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task_manager_node");

    auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>(
        "control_node/control_command");

    if (!controller_client->wait_for_service(2s))
    {
        RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    // RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Cartesian Motion ===");

    // if (!task_utils::activate_controller(node, controller_client, "CartesianMotionController"))
    // {
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // std::vector<double> pose_goal = {0.5, -0.045 ,0.31, 3.14, 0.0, 0.0};
    // if (!task_utils::execute_motion(node, "CartesianMotionController/goal", pose_goal))
    // {
    //     rclcpp::shutdown();
    //     return 1;
    // }

    RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Activate VIC for Contact & Hold  ===");

    // CartesianTrajectoryDianaController VariableImpedanceController
    if (!task_utils::activate_controller(node, controller_client, "VariableImpedanceController"))
    {
        rclcpp::shutdown();
        return 1;
    }

    // std::this_thread::sleep_for(5s);

    std::vector<double> traj_goal = {

    // 先用 10 秒平稳移动到组织上方
    10.0,   0.5,      -0.045,   0.31,     3.14159,   0.0,   0.7853,

    // 再缓慢下探
    20.0,   0.5,      -0.045,   0.244,    3.14159,   0.0,   0.7853,

    // 保持
    25.0,   0.5,      -0.045,   0.244,    3.14159,   0.0,   0.7853,

    // 横向运动
    35.0,   0.5,       0.045,   0.244,    3.14159,   0.0,   0.7853,

    45.0,   0.5,      -0.045,   0.244,    3.14159,   0.0,   0.7853,

    // 抬起
    50.0,   0.5,      -0.045,   0.31,     3.14159,   0.0,   0.7853,
};

    if (!task_utils::execute_motion(node, "VariableImpedanceController/goal", traj_goal))
    {
        rclcpp::shutdown();
        return 1;
    }

    task_utils::deactivate_controller(node, controller_client, "VariableImpedanceController");

    RCLCPP_INFO(node->get_logger(), "=== ALL TASKS COMPLETED SUCCESSFULLY ===");
    rclcpp::shutdown();
    return 0;
}