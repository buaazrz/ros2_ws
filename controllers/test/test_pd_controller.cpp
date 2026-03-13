#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <iostream>

#include "robot_control_msgs/action/robot_motion.hpp"
#include "robot_control_msgs/srv/control_command.hpp"

using namespace std::chrono_literals;
using ACTION = robot_control_msgs::action::RobotMotion;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACTION>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_pd_controller");
  auto client = node->create_client<robot_control_msgs::srv::ControlCommand>("control_node/control_command");

  // ====================== 原有判断1：等待服务 ======================
  if (!client->wait_for_service(2s)) {
    RCLCPP_ERROR(node->get_logger(), "服务不可用");
    rclcpp::shutdown();
    return 1;
  }

  // ====================== 1. 加载控制器（原有逻辑） ======================
  auto load_req = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
  load_req->cmd_name = "load";
  load_req->cmd_params = "controllers::CartesianImpedancePDController";
  auto load_fut = client->async_send_request(load_req);
  // 原有判断2：加载是否成功
  if (rclcpp::spin_until_future_complete(node, load_fut) != rclcpp::FutureReturnCode::SUCCESS || !load_fut.get()->result) {
    RCLCPP_ERROR(node->get_logger(), "加载失败");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "加载成功");

  // ====================== 【新增关键判断】2. 激活控制器 ======================
  auto activate_req = std::make_shared<robot_control_msgs::srv::ControlCommand::Request>();
  activate_req->cmd_name = "activate";
  activate_req->cmd_params = "CartesianImpedancePDController";
  auto activate_fut = client->async_send_request(activate_req);
  // ✅ 判断：激活是否成功（必须！）
  if (rclcpp::spin_until_future_complete(node, activate_fut) != rclcpp::FutureReturnCode::SUCCESS || !activate_fut.get()->result) {
    RCLCPP_ERROR(node->get_logger(), "激活失败");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "激活成功");

  // ====================== 原有判断3：等待Action服务器 ======================
  auto action_client = rclcpp_action::create_client<ACTION>(node, "CartesianImpedancePDController/goal");
  if (!action_client->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Action服务器不可用");
    rclcpp::shutdown();
    return 1;
  }

  // ====================== 发送目标（修改：不再发全0！） ======================
  RCLCPP_INFO(node->get_logger(), "发送目标...");
  auto goal_msg = ACTION::Goal();
  // ✅ 发一个非零目标，机器人一定会动
  goal_msg.target_position.data = {0.0, -0.5, 0.1, 2.0, 0.0, -1.5, 0.0};

  auto send_goal_options = rclcpp_action::Client<ACTION>::SendGoalOptions();
  // 原有判断4：目标是否被接收
  send_goal_options.goal_response_callback = [node](GoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) RCLCPP_ERROR(node->get_logger(), "目标被拒绝");
    else RCLCPP_INFO(node->get_logger(), "目标已接收");
  };

  // 原有判断5：运动结果
  send_goal_options.result_callback = [node](const GoalHandle::WrappedResult & result) {
    if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
      RCLCPP_INFO(node->get_logger(), "运动成功！");
    else
      RCLCPP_ERROR(node->get_logger(), "运动失败");
    rclcpp::shutdown();
  };

  action_client->async_send_goal(goal_msg, send_goal_options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}