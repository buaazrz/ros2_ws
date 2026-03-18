// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_control_msgs:action/RobotMotion.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
#include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_control_msgs/action/detail/robot_motion__functions.h"
#include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `target_position`
#include "std_msgs/msg/float64_multi_array.h"
// Member `target_position`
#include "std_msgs/msg/detail/float64_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_Goal__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_member_array[1] = {
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_Goal, target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_Goal",  // message name
  1,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_Goal),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_Goal__get_type_hash,
  &robot_control_msgs__action__RobotMotion_Goal__get_type_description,
  &robot_control_msgs__action__RobotMotion_Goal__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_Goal)() {
  robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float64MultiArray)();
  if (!robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_Goal__rosidl_typesupport_introspection_c__RobotMotion_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_Result__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_Result",  // message name
  1,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_Result),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_Result__get_type_hash,
  &robot_control_msgs__action__RobotMotion_Result__get_type_description,
  &robot_control_msgs__action__RobotMotion_Result__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_Result)() {
  if (!robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_Result__rosidl_typesupport_introspection_c__RobotMotion_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `current_position`
// already included above
// #include "std_msgs/msg/float64_multi_array.h"
// Member `current_position`
// already included above
// #include "std_msgs/msg/detail/float64_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_Feedback__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_member_array[1] = {
  {
    "current_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_Feedback, current_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_Feedback",  // message name
  1,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_Feedback),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_Feedback__get_type_hash,
  &robot_control_msgs__action__RobotMotion_Feedback__get_type_description,
  &robot_control_msgs__action__RobotMotion_Feedback__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_Feedback)() {
  robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float64MultiArray)();
  if (!robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_Feedback__rosidl_typesupport_introspection_c__RobotMotion_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "robot_control_msgs/action/robot_motion.h"
// Member `goal`
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_SendGoal_Request__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_SendGoal_Request),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_SendGoal_Request__get_type_hash,
  &robot_control_msgs__action__RobotMotion_SendGoal_Request__get_type_description,
  &robot_control_msgs__action__RobotMotion_SendGoal_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Request)() {
  robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_Goal)();
  if (!robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_SendGoal_Response__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_SendGoal_Response),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_SendGoal_Response__get_type_hash,
  &robot_control_msgs__action__RobotMotion_SendGoal_Response__get_type_description,
  &robot_control_msgs__action__RobotMotion_SendGoal_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Response)() {
  robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "robot_control_msgs/action/robot_motion.h"
// Member `request`
// Member `response`
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_SendGoal_Event__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Event__fini(message_memory);
}

size_t robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_SendGoal_Event__request(
  const void * untyped_member)
{
  const robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_SendGoal_Event__request(
  const void * untyped_member, size_t index)
{
  const robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_SendGoal_Event__request(
  void * untyped_member, size_t index)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence * member =
    (robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_SendGoal_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robot_control_msgs__action__RobotMotion_SendGoal_Request * item =
    ((const robot_control_msgs__action__RobotMotion_SendGoal_Request *)
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_SendGoal_Event__request(untyped_member, index));
  robot_control_msgs__action__RobotMotion_SendGoal_Request * value =
    (robot_control_msgs__action__RobotMotion_SendGoal_Request *)(untyped_value);
  *value = *item;
}

void robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_SendGoal_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Request * item =
    ((robot_control_msgs__action__RobotMotion_SendGoal_Request *)
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_SendGoal_Event__request(untyped_member, index));
  const robot_control_msgs__action__RobotMotion_SendGoal_Request * value =
    (const robot_control_msgs__action__RobotMotion_SendGoal_Request *)(untyped_value);
  *item = *value;
}

bool robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_SendGoal_Event__request(
  void * untyped_member, size_t size)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence * member =
    (robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence *)(untyped_member);
  robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence__fini(member);
  return robot_control_msgs__action__RobotMotion_SendGoal_Request__Sequence__init(member, size);
}

size_t robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_SendGoal_Event__response(
  const void * untyped_member)
{
  const robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_SendGoal_Event__response(
  const void * untyped_member, size_t index)
{
  const robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_SendGoal_Event__response(
  void * untyped_member, size_t index)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence * member =
    (robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_SendGoal_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robot_control_msgs__action__RobotMotion_SendGoal_Response * item =
    ((const robot_control_msgs__action__RobotMotion_SendGoal_Response *)
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_SendGoal_Event__response(untyped_member, index));
  robot_control_msgs__action__RobotMotion_SendGoal_Response * value =
    (robot_control_msgs__action__RobotMotion_SendGoal_Response *)(untyped_value);
  *value = *item;
}

void robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_SendGoal_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Response * item =
    ((robot_control_msgs__action__RobotMotion_SendGoal_Response *)
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_SendGoal_Event__response(untyped_member, index));
  const robot_control_msgs__action__RobotMotion_SendGoal_Response * value =
    (const robot_control_msgs__action__RobotMotion_SendGoal_Response *)(untyped_value);
  *item = *value;
}

bool robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_SendGoal_Event__response(
  void * untyped_member, size_t size)
{
  robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence * member =
    (robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence *)(untyped_member);
  robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence__fini(member);
  return robot_control_msgs__action__RobotMotion_SendGoal_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Event, request),  // bytes offset in struct
    NULL,  // default value
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_SendGoal_Event__request,  // size() function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_SendGoal_Event__request,  // get_const(index) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_SendGoal_Event__request,  // get(index) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_SendGoal_Event__request,  // fetch(index, &value) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_SendGoal_Event__request,  // assign(index, value) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_SendGoal_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_SendGoal_Event, response),  // bytes offset in struct
    NULL,  // default value
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_SendGoal_Event__response,  // size() function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_SendGoal_Event__response,  // get_const(index) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_SendGoal_Event__response,  // get(index) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_SendGoal_Event__response,  // fetch(index, &value) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_SendGoal_Event__response,  // assign(index, value) function pointer
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_SendGoal_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_SendGoal_Event",  // message name
  3,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_SendGoal_Event),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_SendGoal_Event__get_type_hash,
  &robot_control_msgs__action__RobotMotion_SendGoal_Event__get_type_description,
  &robot_control_msgs__action__RobotMotion_SendGoal_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Event)() {
  robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Request)();
  robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Response)();
  if (!robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_members = {
  "robot_control_msgs__action",  // service namespace
  "RobotMotion_SendGoal",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_type_support_handle,
  NULL,  // response message
  // robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle
  NULL  // event_message
  // robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle
};


static rosidl_service_type_support_t robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_type_support_handle = {
  0,
  &robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_members,
  get_service_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_SendGoal_Request__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Request_message_type_support_handle,
  &robot_control_msgs__action__RobotMotion_SendGoal_Response__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Response_message_type_support_handle,
  &robot_control_msgs__action__RobotMotion_SendGoal_Event__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_control_msgs,
    action,
    RobotMotion_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_control_msgs,
    action,
    RobotMotion_SendGoal
  ),
  &robot_control_msgs__action__RobotMotion_SendGoal__get_type_hash,
  &robot_control_msgs__action__RobotMotion_SendGoal__get_type_description,
  &robot_control_msgs__action__RobotMotion_SendGoal__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal)(void) {
  if (!robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_SendGoal_Event)()->data;
  }

  return &robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_GetResult_Request__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_GetResult_Request),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_GetResult_Request__get_type_hash,
  &robot_control_msgs__action__RobotMotion_GetResult_Request__get_type_description,
  &robot_control_msgs__action__RobotMotion_GetResult_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Request)() {
  robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "robot_control_msgs/action/robot_motion.h"
// Member `result`
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_GetResult_Response__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_GetResult_Response),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_GetResult_Response__get_type_hash,
  &robot_control_msgs__action__RobotMotion_GetResult_Response__get_type_description,
  &robot_control_msgs__action__RobotMotion_GetResult_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Response)() {
  robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_Result)();
  if (!robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/service_event_info.h"
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "robot_control_msgs/action/robot_motion.h"
// Member `request`
// Member `response`
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_GetResult_Event__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_GetResult_Event__fini(message_memory);
}

size_t robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_GetResult_Event__request(
  const void * untyped_member)
{
  const robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_GetResult_Event__request(
  const void * untyped_member, size_t index)
{
  const robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_GetResult_Event__request(
  void * untyped_member, size_t index)
{
  robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence * member =
    (robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_GetResult_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robot_control_msgs__action__RobotMotion_GetResult_Request * item =
    ((const robot_control_msgs__action__RobotMotion_GetResult_Request *)
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_GetResult_Event__request(untyped_member, index));
  robot_control_msgs__action__RobotMotion_GetResult_Request * value =
    (robot_control_msgs__action__RobotMotion_GetResult_Request *)(untyped_value);
  *value = *item;
}

void robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_GetResult_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robot_control_msgs__action__RobotMotion_GetResult_Request * item =
    ((robot_control_msgs__action__RobotMotion_GetResult_Request *)
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_GetResult_Event__request(untyped_member, index));
  const robot_control_msgs__action__RobotMotion_GetResult_Request * value =
    (const robot_control_msgs__action__RobotMotion_GetResult_Request *)(untyped_value);
  *item = *value;
}

bool robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_GetResult_Event__request(
  void * untyped_member, size_t size)
{
  robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence * member =
    (robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence *)(untyped_member);
  robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence__fini(member);
  return robot_control_msgs__action__RobotMotion_GetResult_Request__Sequence__init(member, size);
}

size_t robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_GetResult_Event__response(
  const void * untyped_member)
{
  const robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_GetResult_Event__response(
  const void * untyped_member, size_t index)
{
  const robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence * member =
    (const robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_GetResult_Event__response(
  void * untyped_member, size_t index)
{
  robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence * member =
    (robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_GetResult_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robot_control_msgs__action__RobotMotion_GetResult_Response * item =
    ((const robot_control_msgs__action__RobotMotion_GetResult_Response *)
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_GetResult_Event__response(untyped_member, index));
  robot_control_msgs__action__RobotMotion_GetResult_Response * value =
    (robot_control_msgs__action__RobotMotion_GetResult_Response *)(untyped_value);
  *value = *item;
}

void robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_GetResult_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robot_control_msgs__action__RobotMotion_GetResult_Response * item =
    ((robot_control_msgs__action__RobotMotion_GetResult_Response *)
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_GetResult_Event__response(untyped_member, index));
  const robot_control_msgs__action__RobotMotion_GetResult_Response * value =
    (const robot_control_msgs__action__RobotMotion_GetResult_Response *)(untyped_value);
  *item = *value;
}

bool robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_GetResult_Event__response(
  void * untyped_member, size_t size)
{
  robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence * member =
    (robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence *)(untyped_member);
  robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence__fini(member);
  return robot_control_msgs__action__RobotMotion_GetResult_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_GetResult_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_GetResult_Event, request),  // bytes offset in struct
    NULL,  // default value
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_GetResult_Event__request,  // size() function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_GetResult_Event__request,  // get_const(index) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_GetResult_Event__request,  // get(index) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_GetResult_Event__request,  // fetch(index, &value) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_GetResult_Event__request,  // assign(index, value) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_GetResult_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_GetResult_Event, response),  // bytes offset in struct
    NULL,  // default value
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__size_function__RobotMotion_GetResult_Event__response,  // size() function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__RobotMotion_GetResult_Event__response,  // get_const(index) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__get_function__RobotMotion_GetResult_Event__response,  // get(index) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__RobotMotion_GetResult_Event__response,  // fetch(index, &value) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__RobotMotion_GetResult_Event__response,  // assign(index, value) function pointer
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__RobotMotion_GetResult_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_GetResult_Event",  // message name
  3,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_GetResult_Event),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_GetResult_Event__get_type_hash,
  &robot_control_msgs__action__RobotMotion_GetResult_Event__get_type_description,
  &robot_control_msgs__action__RobotMotion_GetResult_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Event)() {
  robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Request)();
  robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Response)();
  if (!robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_members = {
  "robot_control_msgs__action",  // service namespace
  "RobotMotion_GetResult",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_type_support_handle,
  NULL,  // response message
  // robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle
  NULL  // event_message
  // robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle
};


static rosidl_service_type_support_t robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_type_support_handle = {
  0,
  &robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_members,
  get_service_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_GetResult_Request__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Request_message_type_support_handle,
  &robot_control_msgs__action__RobotMotion_GetResult_Response__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Response_message_type_support_handle,
  &robot_control_msgs__action__RobotMotion_GetResult_Event__rosidl_typesupport_introspection_c__RobotMotion_GetResult_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_control_msgs,
    action,
    RobotMotion_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_control_msgs,
    action,
    RobotMotion_GetResult
  ),
  &robot_control_msgs__action__RobotMotion_GetResult__get_type_hash,
  &robot_control_msgs__action__RobotMotion_GetResult__get_type_description,
  &robot_control_msgs__action__RobotMotion_GetResult__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult)(void) {
  if (!robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_GetResult_Event)()->data;
  }

  return &robot_control_msgs__action__detail__robot_motion__rosidl_typesupport_introspection_c__RobotMotion_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__functions.h"
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "robot_control_msgs/action/robot_motion.h"
// Member `feedback`
// already included above
// #include "robot_control_msgs/action/detail/robot_motion__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control_msgs__action__RobotMotion_FeedbackMessage__init(message_memory);
}

void robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_fini_function(void * message_memory)
{
  robot_control_msgs__action__RobotMotion_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control_msgs__action__RobotMotion_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_members = {
  "robot_control_msgs__action",  // message namespace
  "RobotMotion_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(robot_control_msgs__action__RobotMotion_FeedbackMessage),
  false,  // has_any_key_member_
  robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_member_array,  // message members
  robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_type_support_handle = {
  0,
  &robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
  &robot_control_msgs__action__RobotMotion_FeedbackMessage__get_type_hash,
  &robot_control_msgs__action__RobotMotion_FeedbackMessage__get_type_description,
  &robot_control_msgs__action__RobotMotion_FeedbackMessage__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_FeedbackMessage)() {
  robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control_msgs, action, RobotMotion_Feedback)();
  if (!robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control_msgs__action__RobotMotion_FeedbackMessage__rosidl_typesupport_introspection_c__RobotMotion_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
