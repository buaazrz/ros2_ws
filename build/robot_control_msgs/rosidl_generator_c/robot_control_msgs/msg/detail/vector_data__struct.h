// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_control_msgs:msg/VectorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/msg/vector_data.h"


#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__STRUCT_H_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/VectorData in the package robot_control_msgs.
typedef struct robot_control_msgs__msg__VectorData
{
  std_msgs__msg__Header header;
  double timestamp;
  rosidl_runtime_c__double__Sequence data;
  rosidl_runtime_c__String name;
} robot_control_msgs__msg__VectorData;

// Struct for a sequence of robot_control_msgs__msg__VectorData.
typedef struct robot_control_msgs__msg__VectorData__Sequence
{
  robot_control_msgs__msg__VectorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control_msgs__msg__VectorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__STRUCT_H_
