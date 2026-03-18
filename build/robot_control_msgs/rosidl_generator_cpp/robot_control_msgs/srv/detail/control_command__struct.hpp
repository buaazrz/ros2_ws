// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_control_msgs:srv/ControlCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/srv/control_command.hpp"


#ifndef ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__STRUCT_HPP_
#define ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_control_msgs__srv__ControlCommand_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_control_msgs__srv__ControlCommand_Request __declspec(deprecated)
#endif

namespace robot_control_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlCommand_Request_
{
  using Type = ControlCommand_Request_<ContainerAllocator>;

  explicit ControlCommand_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_name = "";
      this->cmd_params = "";
    }
  }

  explicit ControlCommand_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_name(_alloc),
    cmd_params(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd_name = "";
      this->cmd_params = "";
    }
  }

  // field types and members
  using _cmd_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cmd_name_type cmd_name;
  using _cmd_params_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cmd_params_type cmd_params;

  // setters for named parameter idiom
  Type & set__cmd_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cmd_name = _arg;
    return *this;
  }
  Type & set__cmd_params(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cmd_params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control_msgs__srv__ControlCommand_Request
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control_msgs__srv__ControlCommand_Request
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlCommand_Request_ & other) const
  {
    if (this->cmd_name != other.cmd_name) {
      return false;
    }
    if (this->cmd_params != other.cmd_params) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlCommand_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlCommand_Request_

// alias to use template instance with default allocator
using ControlCommand_Request =
  robot_control_msgs::srv::ControlCommand_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_control_msgs


#ifndef _WIN32
# define DEPRECATED__robot_control_msgs__srv__ControlCommand_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_control_msgs__srv__ControlCommand_Response __declspec(deprecated)
#endif

namespace robot_control_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlCommand_Response_
{
  using Type = ControlCommand_Response_<ContainerAllocator>;

  explicit ControlCommand_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit ControlCommand_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control_msgs__srv__ControlCommand_Response
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control_msgs__srv__ControlCommand_Response
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlCommand_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlCommand_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlCommand_Response_

// alias to use template instance with default allocator
using ControlCommand_Response =
  robot_control_msgs::srv::ControlCommand_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_control_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_control_msgs__srv__ControlCommand_Event __attribute__((deprecated))
#else
# define DEPRECATED__robot_control_msgs__srv__ControlCommand_Event __declspec(deprecated)
#endif

namespace robot_control_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlCommand_Event_
{
  using Type = ControlCommand_Event_<ContainerAllocator>;

  explicit ControlCommand_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ControlCommand_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_control_msgs::srv::ControlCommand_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_control_msgs::srv::ControlCommand_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_control_msgs__srv__ControlCommand_Event
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_control_msgs__srv__ControlCommand_Event
    std::shared_ptr<robot_control_msgs::srv::ControlCommand_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlCommand_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlCommand_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlCommand_Event_

// alias to use template instance with default allocator
using ControlCommand_Event =
  robot_control_msgs::srv::ControlCommand_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_control_msgs

namespace robot_control_msgs
{

namespace srv
{

struct ControlCommand
{
  using Request = robot_control_msgs::srv::ControlCommand_Request;
  using Response = robot_control_msgs::srv::ControlCommand_Response;
  using Event = robot_control_msgs::srv::ControlCommand_Event;
};

}  // namespace srv

}  // namespace robot_control_msgs

#endif  // ROBOT_CONTROL_MSGS__SRV__DETAIL__CONTROL_COMMAND__STRUCT_HPP_
