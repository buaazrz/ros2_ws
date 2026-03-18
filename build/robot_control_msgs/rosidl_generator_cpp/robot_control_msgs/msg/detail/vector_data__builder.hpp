// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control_msgs:msg/VectorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/msg/vector_data.hpp"


#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__BUILDER_HPP_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control_msgs/msg/detail/vector_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control_msgs
{

namespace msg
{

namespace builder
{

class Init_VectorData_name
{
public:
  explicit Init_VectorData_name(::robot_control_msgs::msg::VectorData & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::msg::VectorData name(::robot_control_msgs::msg::VectorData::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::msg::VectorData msg_;
};

class Init_VectorData_data
{
public:
  explicit Init_VectorData_data(::robot_control_msgs::msg::VectorData & msg)
  : msg_(msg)
  {}
  Init_VectorData_name data(::robot_control_msgs::msg::VectorData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_VectorData_name(msg_);
  }

private:
  ::robot_control_msgs::msg::VectorData msg_;
};

class Init_VectorData_timestamp
{
public:
  explicit Init_VectorData_timestamp(::robot_control_msgs::msg::VectorData & msg)
  : msg_(msg)
  {}
  Init_VectorData_data timestamp(::robot_control_msgs::msg::VectorData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_VectorData_data(msg_);
  }

private:
  ::robot_control_msgs::msg::VectorData msg_;
};

class Init_VectorData_header
{
public:
  Init_VectorData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VectorData_timestamp header(::robot_control_msgs::msg::VectorData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VectorData_timestamp(msg_);
  }

private:
  ::robot_control_msgs::msg::VectorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::msg::VectorData>()
{
  return robot_control_msgs::msg::builder::Init_VectorData_header();
}

}  // namespace robot_control_msgs

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__BUILDER_HPP_
