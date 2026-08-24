#include <robot_controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include "robot_control_msgs/action/robot_motion.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
using namespace std::chrono_literals;

using ACTION = robot_control_msgs::action::RobotMotion;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_trajectory_controller_zrz");
  auto client = rclcpp_action::create_client<ACTION>(node, "CartesianTrajImpPDController/goal");

  

  if (!client->wait_for_action_server())
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return 1;
  }
  auto goal_msg = ACTION::Goal();
  goal_msg.target_position.data = {
    25,  0.5, -0.045 ,0.31, 3.14, 0.0, 0.0, 
    35,  0.5, -0.045  ,0.244, 3.14, 0.0, 0.0, 
    45,  0.5, 0.045  ,0.244, 3.14, 0.0, 0.0,
    55,  0.5, -0.045  ,0.244, 3.14, 0.0, 0.0,
    65,  0.5, -0.045  ,0.31, 3.14, 0.0, 0.0,

 };
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
  auto result_future = client->async_get_result(handle);
  result = rclcpp::spin_until_future_complete(node, result_future);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(node->get_logger(), "Failed to get result");
      rclcpp::shutdown();
      return 0;
  }
  result_future.get().result->success;
  rclcpp::shutdown();
  return 0;
}


