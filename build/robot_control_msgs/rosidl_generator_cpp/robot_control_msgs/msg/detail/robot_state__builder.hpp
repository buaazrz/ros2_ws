// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/msg/robot_state.hpp"


#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_robot_state
{
public:
  explicit Init_RobotState_robot_state(::robot_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::msg::RobotState robot_state(::robot_control_msgs::msg::RobotState::_robot_state_type arg)
  {
    msg_.robot_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_header
{
public:
  Init_RobotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_robot_state header(::robot_control_msgs::msg::RobotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotState_robot_state(msg_);
  }

private:
  ::robot_control_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::msg::RobotState>()
{
  return robot_control_msgs::msg::builder::Init_RobotState_header();
}

}  // namespace robot_control_msgs

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
