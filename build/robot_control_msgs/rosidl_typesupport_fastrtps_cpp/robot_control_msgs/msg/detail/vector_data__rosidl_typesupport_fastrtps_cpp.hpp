// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from robot_control_msgs:msg/VectorData.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_control_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "robot_control_msgs/msg/detail/vector_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace robot_control_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
cdr_serialize(
  const robot_control_msgs::msg::VectorData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robot_control_msgs::msg::VectorData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
get_serialized_size(
  const robot_control_msgs::msg::VectorData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
max_serialized_size_VectorData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
cdr_serialize_key(
  const robot_control_msgs::msg::VectorData & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
get_serialized_size_key(
  const robot_control_msgs::msg::VectorData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
max_serialized_size_key_VectorData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robot_control_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_control_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_control_msgs, msg, VectorData)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
