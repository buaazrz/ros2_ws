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

    // 创建通用控制器切换的 Service Client
    auto controller_client = node->create_client<robot_control_msgs::srv::ControlCommand>(
        "control_node/control_command");
        
    if (!controller_client->wait_for_service(1s)) {
        RCLCPP_ERROR(node->get_logger(), "Service not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    // =========================================================
    // 阶段 1: 使用 CartesianMotionController 到达初始准备位姿
    // =========================================================
    RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Cartesian Motion ===");
    
    if (!task_utils::activate_controller(node, controller_client, "CartesianMotionController")) {
        rclcpp::shutdown(); return 1;
    }

    std::vector<double> pose_goal = {0.536320, -0.163060, 0.356040, 0.003903, 0.022033, -0.164466}; 
    if (!task_utils::execute_motion(node, "CartesianMotionController/goal", pose_goal)) {
        rclcpp::shutdown(); return 1;
    }

    std::this_thread::sleep_for(5s);

    if (!task_utils::activate_controller(node, controller_client, "CartesianTrajectoryDianaController")) return 1;

    // 构造一段轨迹：从当前准备点，向下走 10cm 到 0.2600，设定时间为 10 秒
    // 这样向下的速度就是 10cm / 10s = 1 cm/s，这是比较安全的探寻速度
    std::vector<double> init_traj_goal = {
        10, 0.536320, -0.163060, 0.286040, 0.003903, 0.022033, -0.164466
    };

    std::this_thread::sleep_for(5s);


    // 执行！机械臂会以 1cm/s 向下走，一旦中途碰到东西力超过15N，就会立刻停下，下面的 if 就会通过
    if (task_utils::execute_motion(node, "CartesianTrajectoryDianaController/goal", init_traj_goal)) {
        RCLCPP_INFO(node->get_logger(), "碰到了！力检测已触发，机械臂已停止。");
    }

    // =========================================================
    // 阶段 2: 切换为 CartesianTrajectoryController 执行轨迹
    // =========================================================
    std::this_thread::sleep_for(5s);

    RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Cartesian Trajectory ===");
    
    if (!task_utils::activate_controller(node, controller_client, "VariableImpedanceController")) {
        rclcpp::shutdown(); return 1;
    }

    std::vector<double> traj_goal = {
        5,  0.536320, -0.163060, 0.324040, 0.003903, 0.022033, -0.164466, 
        10, 0.536320, -0.0500, 0.324040,0.003903, 0.022033, -0.164466,  
        20, 0.536320, -0.163060, 0.324040, 0.003903, 0.022033, -0.164466,  
    };

    std::this_thread::sleep_for(5s);

    
    if (!task_utils::execute_motion(node, "VariableImpedanceController/goal", traj_goal)) {
        rclcpp::shutdown(); return 1;
    }

    // =========================================================
    // 阶段 3: 任务结束，失活所有已调用的控制器
    // =========================================================
    RCLCPP_INFO(node->get_logger(), "=== PHASE 3: Deactivating Controllers ===");

    task_utils::deactivate_controller(node, controller_client, "VariableImpedanceController");

    RCLCPP_INFO(node->get_logger(), "=== ALL TASKS COMPLETED SUCCESSFULLY ===");
    rclcpp::shutdown();
    return 0;
}