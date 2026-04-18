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
    {0.388093, -0.017484, 0.563496, -0.070626, -0.036796, 1.237924},
    {0.393978, -0.020376, 0.582013, -0.023745, -0.104658, -0.116363},
    {0.383343, -0.055589, 0.564219, 0.059458, -0.050794, -1.582844},
    {0.382006, -0.023352, 0.557842, 0.058523, 0.097837, -2.819715},
    {0.396152, 0.076642, 0.700689, 1.571252, -0.002491, -0.245396},
    {0.393978, -0.020376, 0.582013, -0.023745, -0.104658, -0.116363},
    {0.368437, -0.150462, 0.723017, -1.612136, 0.128530, -0.005142},
    {0.536373, -0.053360, 0.757816, -0.090827, -1.536739, -0.064624},
    {0.328020, -0.030965, 0.778537, 0.104046, 3.134241, -0.065001},
    {0.259296, -0.025791, 0.563719, -0.001966, 1.547871, -0.114077},
    {0.384902, -0.124681, 0.530380, -0.573092, 0.522604, -0.106241},
    {0.427223, 0.005881, 0.532765, 0.728829, 0.285744, -0.283229},
    {0.439865, -0.062277, 0.540892, -0.402850, -0.177149, 0.066258},
    {0.421340, 0.012725, 0.577212, 0.613137, -0.396028, -0.411743},
    {0.421067, -0.066942, 0.769763, 2.434913, -0.346519, -0.731203},
    {0.387140, -0.200508, 0.710622, -2.074333, 0.121336, 0.425728},
    {0.444053, -0.036860, 0.576507, -0.019252, -0.544277, -0.094298},
    {0.363240, -0.041425, 0.763540, -0.115021, -2.102144, -0.089656},
    {0.284164, -0.023156, 0.555944, 0.054386, 0.738426, -0.087167},
    {0.404762, 0.036411, 0.565618, 0.700547, -0.011118, -0.252597},
    {0.395931, -0.076534, 0.564256, -0.678410, 0.045076, 0.111930},
  };

  // 2. 自动生成包含时间戳和“停留”逻辑的完整轨迹数组
  std::vector<double> full_trajectory_data;
  double current_time = 0.0;
  
  // 你可以在这里调节运动节奏
  double move_duration = 5.0; // 每一段平移耗时 5 秒
  double stay_duration = 3.0; // 到达每个点后停留 2 秒

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