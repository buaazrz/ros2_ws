// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_control_msgs:srv/ControlCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/srv/control_command.hpp"


#ifndef ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__TRAITS_HPP_
#define ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_control_msgs/srv/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_control_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ControlCommand_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_name
  {
    out << "cmd_name: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_name, out);
    out << ", ";
  }

  // member: cmd_params
  {
    out << "cmd_params: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_params, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_name: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_name, out);
    out << "\n";
  }

  // member: cmd_params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_params: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_params, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlCommand_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_control_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_control_msgs::srv::ControlCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_control_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_control_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_control_msgs::srv::ControlCommand_Request & msg)
{
  return robot_control_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_control_msgs::srv::ControlCommand_Request>()
{
  return "robot_control_msgs::srv::ControlCommand_Request";
}

template<>
inline const char * name<robot_control_msgs::srv::ControlCommand_Request>()
{
  return "robot_control_msgs/srv/ControlCommand_Request";
}

template<>
struct has_fixed_size<robot_control_msgs::srv::ControlCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_control_msgs::srv::ControlCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_control_msgs::srv::ControlCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_control_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ControlCommand_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlCommand_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_control_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_control_msgs::srv::ControlCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_control_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_control_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_control_msgs::srv::ControlCommand_Response & msg)
{
  return robot_control_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_control_msgs::srv::ControlCommand_Response>()
{
  return "robot_control_msgs::srv::ControlCommand_Response";
}

template<>
inline const char * name<robot_control_msgs::srv::ControlCommand_Response>()
{
  return "robot_control_msgs/srv/ControlCommand_Response";
}

template<>
struct has_fixed_size<robot_control_msgs::srv::ControlCommand_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_control_msgs::srv::ControlCommand_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_control_msgs::srv::ControlCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace robot_control_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ControlCommand_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlCommand_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlCommand_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_control_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_control_msgs::srv::ControlCommand_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_control_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_control_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_control_msgs::srv::ControlCommand_Event & msg)
{
  return robot_control_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_control_msgs::srv::ControlCommand_Event>()
{
  return "robot_control_msgs::srv::ControlCommand_Event";
}

template<>
inline const char * name<robot_control_msgs::srv::ControlCommand_Event>()
{
  return "robot_control_msgs/srv/ControlCommand_Event";
}

template<>
struct has_fixed_size<robot_control_msgs::srv::ControlCommand_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_control_msgs::srv::ControlCommand_Event>
  : std::integral_constant<bool, has_bounded_size<robot_control_msgs::srv::ControlCommand_Request>::value && has_bounded_size<robot_control_msgs::srv::ControlCommand_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<robot_control_msgs::srv::ControlCommand_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_control_msgs::srv::ControlCommand>()
{
  return "robot_control_msgs::srv::ControlCommand";
}

template<>
inline const char * name<robot_control_msgs::srv::ControlCommand>()
{
  return "robot_control_msgs/srv/ControlCommand";
}

template<>
struct has_fixed_size<robot_control_msgs::srv::ControlCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_control_msgs::srv::ControlCommand_Request>::value &&
    has_fixed_size<robot_control_msgs::srv::ControlCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_control_msgs::srv::ControlCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_control_msgs::srv::ControlCommand_Request>::value &&
    has_bounded_size<robot_control_msgs::srv::ControlCommand_Response>::value
  >
{
};

template<>
struct is_service<robot_control_msgs::srv::ControlCommand>
  : std::true_type
{
};

template<>
struct is_service_request<robot_control_msgs::srv::ControlCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_control_msgs::srv::ControlCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__TRAITS_HPP_
