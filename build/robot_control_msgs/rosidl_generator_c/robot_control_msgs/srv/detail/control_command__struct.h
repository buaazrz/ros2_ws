// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_control_msgs:srv/ControlCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/srv/control_command.h"


#ifndef ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__STRUCT_H_
#define ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd_name'
// Member 'cmd_params'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ControlCommand in the package robot_control_msgs.
typedef struct robot_control_msgs__srv__ControlCommand_Request
{
  rosidl_runtime_c__String cmd_name;
  rosidl_runtime_c__String cmd_params;
} robot_control_msgs__srv__ControlCommand_Request;

// Struct for a sequence of robot_control_msgs__srv__ControlCommand_Request.
typedef struct robot_control_msgs__srv__ControlCommand_Request__Sequence
{
  robot_control_msgs__srv__ControlCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control_msgs__srv__ControlCommand_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/ControlCommand in the package robot_control_msgs.
typedef struct robot_control_msgs__srv__ControlCommand_Response
{
  bool result;
} robot_control_msgs__srv__ControlCommand_Response;

// Struct for a sequence of robot_control_msgs__srv__ControlCommand_Response.
typedef struct robot_control_msgs__srv__ControlCommand_Response__Sequence
{
  robot_control_msgs__srv__ControlCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control_msgs__srv__ControlCommand_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_control_msgs__srv__ControlCommand_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_control_msgs__srv__ControlCommand_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/ControlCommand in the package robot_control_msgs.
typedef struct robot_control_msgs__srv__ControlCommand_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_control_msgs__srv__ControlCommand_Request__Sequence request;
  robot_control_msgs__srv__ControlCommand_Response__Sequence response;
} robot_control_msgs__srv__ControlCommand_Event;

// Struct for a sequence of robot_control_msgs__srv__ControlCommand_Event.
typedef struct robot_control_msgs__srv__ControlCommand_Event__Sequence
{
  robot_control_msgs__srv__ControlCommand_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control_msgs__srv__ControlCommand_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__STRUCT_H_
