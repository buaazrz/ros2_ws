// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from robot_control_msgs:msg/VectorData.idl
// generated code does not contain a copyright notice
#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_control_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_control_msgs/msg/detail/vector_data__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
bool cdr_serialize_robot_control_msgs__msg__VectorData(
  const robot_control_msgs__msg__VectorData * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
bool cdr_deserialize_robot_control_msgs__msg__VectorData(
  eprosima::fastcdr::Cdr &,
  robot_control_msgs__msg__VectorData * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
size_t get_serialized_size_robot_control_msgs__msg__VectorData(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
size_t max_serialized_size_robot_control_msgs__msg__VectorData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
bool cdr_serialize_key_robot_control_msgs__msg__VectorData(
  const robot_control_msgs__msg__VectorData * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
size_t get_serialized_size_key_robot_control_msgs__msg__VectorData(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
size_t max_serialized_size_key_robot_control_msgs__msg__VectorData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_control_msgs, msg, VectorData)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
