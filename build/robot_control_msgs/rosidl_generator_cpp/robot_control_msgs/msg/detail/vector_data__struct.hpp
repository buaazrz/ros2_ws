// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_control_msgs:msg/VectorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/msg/vector_data.hpp"


#ifndef ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__STRUCT_HPP_
#define ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_control_msgs__msg__VectorData __attribute__((deprecated))
#else
# define DEPRECATED__robot_control_msgs__msg__VectorData __declspec(deprecated)
#endif

namespace robot_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VectorData_
{
  using Type = VectorData_<ContainerAllocator>;

  explicit VectorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0;
      this->name = "";
    }
  }

  explicit VectorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0.0;
      this->name = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _data_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _data_type data;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control_msgs::msg::VectorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control_msgs::msg::VectorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::msg::VectorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::msg::VectorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control_msgs__msg__VectorData
    std::shared_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control_msgs__msg__VectorData
    std::shared_ptr<robot_control_msgs::msg::VectorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VectorData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const VectorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VectorData_

// alias to use template instance with default allocator
using VectorData =
  robot_control_msgs::msg::VectorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_control_msgs

#endif  // ROBOT_CONTROL_MSGS__MSG__DETAIL__VECTOR_DATA__STRUCT_HPP_
