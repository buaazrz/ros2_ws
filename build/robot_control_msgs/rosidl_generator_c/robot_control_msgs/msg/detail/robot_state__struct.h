// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/msg/robot_state.h"


#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

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
// Member 'robot_state'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RobotState in the package robot_control_msgs.
typedef struct robot_control_msgs__msg__RobotState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence robot_state;
} robot_control_msgs__msg__RobotState;

// Struct for a sequence of robot_control_msgs__msg__RobotState.
typedef struct robot_control_msgs__msg__RobotState__Sequence
{
  robot_control_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
