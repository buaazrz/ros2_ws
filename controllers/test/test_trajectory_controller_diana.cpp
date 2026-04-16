#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <vector>
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using ACTION = robot_control_msgs::action::RobotMotion;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_trajectory_controller_diana");
  auto client = rclcpp_action::create_client<ACTION>(node, "CartesianTrajectoryDianaController/goal");

  if (!client->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return 1;
  }

  // 1. 在这里只管罗列你的纯位姿数据 (N个位姿，不需要手敲时间戳)
  std::vector<std::vector<double>> target_poses = {
    {0.5500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100},
    {0.5500, -0.0000, 0.3600, 0.0000, -0.0000, -0.3100},
    {0.5500,  0.1800, 0.3600, 0.0000, -0.0000, -0.3100},
    {0.4500,  0.1800, 0.3600, 0.0000, -0.0000, -0.3100},
    {0.4500, -0.0000, 0.3600, 0.0000, -0.0000, -0.3100},
    {0.4500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100},
    {0.5500, -0.1800, 0.3600, 0.0000, -0.0000, -0.3100}
  };

  // 2. 自动生成包含时间戳和“停留”逻辑的完整轨迹数组
  std::vector<double> full_trajectory_data;
  double current_time = 0.0;
  
  // 你可以在这里调节运动节奏
  double move_duration = 5.0; // 每一段平移耗时 5 秒
  double stay_duration = 2.0; // 到达每个点后停留 2 秒

  for (const auto& pose : target_poses)
  {
      // 动作 A：平滑移动到当前目标位姿
      current_time += move_duration;
      full_trajectory_data.push_back(current_time);
      full_trajectory_data.insert(full_trajectory_data.end(), pose.begin(), pose.end());

      // 动作 B：在原地停留 (时间推进了，但位姿不变)
      current_time += stay_duration;
      full_trajectory_data.push_back(current_time);
      full_trajectory_data.insert(full_trajectory_data.end(), pose.begin(), pose.end());
  }

  // 3. 打包发送给控制器
  auto goal_msg = ACTION::Goal();
  goal_msg.target_position.data = full_trajectory_data;

  RCLCPP_INFO(node->get_logger(), "Sending trajectory with %zu target poses. Total time: %.1f seconds.", 
              target_poses.size(), current_time);

  auto handle_future = client->async_send_goal(goal_msg);
  auto result = rclcpp::spin_until_future_complete(node, handle_future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    rclcpp::shutdown();
    return 0;
  }
  
  auto handle = handle_future.get();
  if(handle == nullptr)
  {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected");
    rclcpp::shutdown();
    return 0;
  }
  
  RCLCPP_INFO(node->get_logger(), "Goal accepted, waiting for execution to finish...");
  
  auto result_future = client->async_get_result(handle);
  result = rclcpp::spin_until_future_complete(node, result_future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result");
      rclcpp::shutdown();
      return 0;
  }
  
  if (result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node->get_logger(), "Trajectory execution succeeded!");
  } else {
      RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed or was canceled.");
  }

  rclcpp::shutdown();
  return 0;
}