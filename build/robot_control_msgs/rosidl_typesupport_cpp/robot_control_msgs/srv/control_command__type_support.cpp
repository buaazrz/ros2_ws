// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from robot_control_msgs:srv/ControlCommand.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_control_msgs/srv/detail/control_command__functions.h"
#include "robot_control_msgs/srv/detail/control_command__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robot_control_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlCommand_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlCommand_Request_type_support_ids_t;

static const _ControlCommand_Request_type_support_ids_t _ControlCommand_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlCommand_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlCommand_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlCommand_Request_type_support_symbol_names_t _ControlCommand_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_control_msgs, srv, ControlCommand_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_control_msgs, srv, ControlCommand_Request)),
  }
};

typedef struct _ControlCommand_Request_type_support_data_t
{
  void * data[2];
} _ControlCommand_Request_type_support_data_t;

static _ControlCommand_Request_type_support_data_t _ControlCommand_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlCommand_Request_message_typesupport_map = {
  2,
  "robot_control_msgs",
  &_ControlCommand_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ControlCommand_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ControlCommand_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ControlCommand_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlCommand_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_control_msgs__srv__ControlCommand_Request__get_type_hash,
  &robot_control_msgs__srv__ControlCommand_Request__get_type_description,
  &robot_control_msgs__srv__ControlCommand_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Request>()
{
  return &::robot_control_msgs::srv::rosidl_typesupport_cpp::ControlCommand_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_control_msgs, srv, ControlCommand_Request)() {
  return get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_control_msgs/srv/detail/control_command__functions.h"
// already included above
// #include "robot_control_msgs/srv/detail/control_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_control_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlCommand_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlCommand_Response_type_support_ids_t;

static const _ControlCommand_Response_type_support_ids_t _ControlCommand_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlCommand_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlCommand_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlCommand_Response_type_support_symbol_names_t _ControlCommand_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_control_msgs, srv, ControlCommand_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_control_msgs, srv, ControlCommand_Response)),
  }
};

typedef struct _ControlCommand_Response_type_support_data_t
{
  void * data[2];
} _ControlCommand_Response_type_support_data_t;

static _ControlCommand_Response_type_support_data_t _ControlCommand_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlCommand_Response_message_typesupport_map = {
  2,
  "robot_control_msgs",
  &_ControlCommand_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ControlCommand_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ControlCommand_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ControlCommand_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlCommand_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_control_msgs__srv__ControlCommand_Response__get_type_hash,
  &robot_control_msgs__srv__ControlCommand_Response__get_type_description,
  &robot_control_msgs__srv__ControlCommand_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Response>()
{
  return &::robot_control_msgs::srv::rosidl_typesupport_cpp::ControlCommand_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_control_msgs, srv, ControlCommand_Response)() {
  return get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_control_msgs/srv/detail/control_command__functions.h"
// already included above
// #include "robot_control_msgs/srv/detail/control_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_control_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlCommand_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlCommand_Event_type_support_ids_t;

static const _ControlCommand_Event_type_support_ids_t _ControlCommand_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlCommand_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlCommand_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlCommand_Event_type_support_symbol_names_t _ControlCommand_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_control_msgs, srv, ControlCommand_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_control_msgs, srv, ControlCommand_Event)),
  }
};

typedef struct _ControlCommand_Event_type_support_data_t
{
  void * data[2];
} _ControlCommand_Event_type_support_data_t;

static _ControlCommand_Event_type_support_data_t _ControlCommand_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlCommand_Event_message_typesupport_map = {
  2,
  "robot_control_msgs",
  &_ControlCommand_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ControlCommand_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ControlCommand_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ControlCommand_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlCommand_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_control_msgs__srv__ControlCommand_Event__get_type_hash,
  &robot_control_msgs__srv__ControlCommand_Event__get_type_description,
  &robot_control_msgs__srv__ControlCommand_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Event>()
{
  return &::robot_control_msgs::srv::rosidl_typesupport_cpp::ControlCommand_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_control_msgs, srv, ControlCommand_Event)() {
  return get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "robot_control_msgs/srv/detail/control_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_control_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ControlCommand_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ControlCommand_type_support_ids_t;

static const _ControlCommand_type_support_ids_t _ControlCommand_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ControlCommand_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ControlCommand_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ControlCommand_type_support_symbol_names_t _ControlCommand_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_control_msgs, srv, ControlCommand)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_control_msgs, srv, ControlCommand)),
  }
};

typedef struct _ControlCommand_type_support_data_t
{
  void * data[2];
} _ControlCommand_type_support_data_t;

static _ControlCommand_type_support_data_t _ControlCommand_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ControlCommand_service_typesupport_map = {
  2,
  "robot_control_msgs",
  &_ControlCommand_service_typesupport_ids.typesupport_identifier[0],
  &_ControlCommand_service_typesupport_symbol_names.symbol_name[0],
  &_ControlCommand_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ControlCommand_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ControlCommand_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_control_msgs::srv::ControlCommand_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<robot_control_msgs::srv::ControlCommand>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<robot_control_msgs::srv::ControlCommand>,
  &robot_control_msgs__srv__ControlCommand__get_type_hash,
  &robot_control_msgs__srv__ControlCommand__get_type_description,
  &robot_control_msgs__srv__ControlCommand__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robot_control_msgs::srv::ControlCommand>()
{
  return &::robot_control_msgs::srv::rosidl_typesupport_cpp::ControlCommand_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_control_msgs, srv, ControlCommand)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robot_control_msgs::srv::ControlCommand>();
}

#ifdef __cplusplus
}
#endif
