// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control_msgs:srv/ControlCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/srv/control_command.hpp"


#ifndef ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
#define ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control_msgs/srv/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control_msgs
{

namespace srv
{

namespace builder
{

class Init_ControlCommand_Request_cmd_params
{
public:
  explicit Init_ControlCommand_Request_cmd_params(::robot_control_msgs::srv::ControlCommand_Request & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::srv::ControlCommand_Request cmd_params(::robot_control_msgs::srv::ControlCommand_Request::_cmd_params_type arg)
  {
    msg_.cmd_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::srv::ControlCommand_Request msg_;
};

class Init_ControlCommand_Request_cmd_name
{
public:
  Init_ControlCommand_Request_cmd_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlCommand_Request_cmd_params cmd_name(::robot_control_msgs::srv::ControlCommand_Request::_cmd_name_type arg)
  {
    msg_.cmd_name = std::move(arg);
    return Init_ControlCommand_Request_cmd_params(msg_);
  }

private:
  ::robot_control_msgs::srv::ControlCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::srv::ControlCommand_Request>()
{
  return robot_control_msgs::srv::builder::Init_ControlCommand_Request_cmd_name();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace srv
{

namespace builder
{

class Init_ControlCommand_Response_result
{
public:
  Init_ControlCommand_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_control_msgs::srv::ControlCommand_Response result(::robot_control_msgs::srv::ControlCommand_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::srv::ControlCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::srv::ControlCommand_Response>()
{
  return robot_control_msgs::srv::builder::Init_ControlCommand_Response_result();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace srv
{

namespace builder
{

class Init_ControlCommand_Event_response
{
public:
  explicit Init_ControlCommand_Event_response(::robot_control_msgs::srv::ControlCommand_Event & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::srv::ControlCommand_Event response(::robot_control_msgs::srv::ControlCommand_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::srv::ControlCommand_Event msg_;
};

class Init_ControlCommand_Event_request
{
public:
  explicit Init_ControlCommand_Event_request(::robot_control_msgs::srv::ControlCommand_Event & msg)
  : msg_(msg)
  {}
  Init_ControlCommand_Event_response request(::robot_control_msgs::srv::ControlCommand_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ControlCommand_Event_response(msg_);
  }

private:
  ::robot_control_msgs::srv::ControlCommand_Event msg_;
};

class Init_ControlCommand_Event_info
{
public:
  Init_ControlCommand_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlCommand_Event_request info(::robot_control_msgs::srv::ControlCommand_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ControlCommand_Event_request(msg_);
  }

private:
  ::robot_control_msgs::srv::ControlCommand_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::srv::ControlCommand_Event>()
{
  return robot_control_msgs::srv::builder::Init_ControlCommand_Event_info();
}

}  // namespace robot_control_msgs

#endif  // ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
