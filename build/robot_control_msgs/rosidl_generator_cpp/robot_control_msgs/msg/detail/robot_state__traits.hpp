// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/msg/robot_state.hpp"


#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_control_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace robot_control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: robot_state
  {
    if (msg.robot_state.size() == 0) {
      out << "robot_state: []";
    } else {
      out << "robot_state: [";
      size_t pending_items = msg.robot_state.size();
      for (auto item : msg.robot_state) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: robot_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_state.size() == 0) {
      out << "robot_state: []\n";
    } else {
      out << "robot_state:\n";
      for (auto item : msg.robot_state) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_control_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_control_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_control_msgs::msg::RobotState & msg)
{
  return robot_control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_control_msgs::msg::RobotState>()
{
  return "robot_control_msgs::msg::RobotState";
}

template<>
inline const char * name<robot_control_msgs::msg::RobotState>()
{
  return "robot_control_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<robot_control_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_control_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_control_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
