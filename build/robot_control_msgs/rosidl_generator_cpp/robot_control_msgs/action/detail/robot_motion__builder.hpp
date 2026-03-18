// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_control_msgs:action/RobotMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control_msgs/action/robot_motion.hpp"


#ifndef ROBOT_CONTROL_MSGS__ACTION__DETAIL__ROBOT_MOTION__BUILDER_HPP_
#define ROBOT_CONTROL_MSGS__ACTION__DETAIL__ROBOT_MOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_control_msgs/action/detail/robot_motion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_Goal_target_position
{
public:
  Init_RobotMotion_Goal_target_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_control_msgs::action::RobotMotion_Goal target_position(::robot_control_msgs::action::RobotMotion_Goal::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_Goal>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_Goal_target_position();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_Result_success
{
public:
  Init_RobotMotion_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_control_msgs::action::RobotMotion_Result success(::robot_control_msgs::action::RobotMotion_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_Result>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_Result_success();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_Feedback_current_position
{
public:
  Init_RobotMotion_Feedback_current_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_control_msgs::action::RobotMotion_Feedback current_position(::robot_control_msgs::action::RobotMotion_Feedback::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_Feedback>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_Feedback_current_position();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_SendGoal_Request_goal
{
public:
  explicit Init_RobotMotion_SendGoal_Request_goal(::robot_control_msgs::action::RobotMotion_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::action::RobotMotion_SendGoal_Request goal(::robot_control_msgs::action::RobotMotion_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Request msg_;
};

class Init_RobotMotion_SendGoal_Request_goal_id
{
public:
  Init_RobotMotion_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotMotion_SendGoal_Request_goal goal_id(::robot_control_msgs::action::RobotMotion_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RobotMotion_SendGoal_Request_goal(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_SendGoal_Request>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_SendGoal_Request_goal_id();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_SendGoal_Response_stamp
{
public:
  explicit Init_RobotMotion_SendGoal_Response_stamp(::robot_control_msgs::action::RobotMotion_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::action::RobotMotion_SendGoal_Response stamp(::robot_control_msgs::action::RobotMotion_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Response msg_;
};

class Init_RobotMotion_SendGoal_Response_accepted
{
public:
  Init_RobotMotion_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotMotion_SendGoal_Response_stamp accepted(::robot_control_msgs::action::RobotMotion_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_RobotMotion_SendGoal_Response_stamp(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_SendGoal_Response>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_SendGoal_Response_accepted();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_SendGoal_Event_response
{
public:
  explicit Init_RobotMotion_SendGoal_Event_response(::robot_control_msgs::action::RobotMotion_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::action::RobotMotion_SendGoal_Event response(::robot_control_msgs::action::RobotMotion_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Event msg_;
};

class Init_RobotMotion_SendGoal_Event_request
{
public:
  explicit Init_RobotMotion_SendGoal_Event_request(::robot_control_msgs::action::RobotMotion_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_RobotMotion_SendGoal_Event_response request(::robot_control_msgs::action::RobotMotion_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RobotMotion_SendGoal_Event_response(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Event msg_;
};

class Init_RobotMotion_SendGoal_Event_info
{
public:
  Init_RobotMotion_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotMotion_SendGoal_Event_request info(::robot_control_msgs::action::RobotMotion_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RobotMotion_SendGoal_Event_request(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_SendGoal_Event>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_SendGoal_Event_info();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_GetResult_Request_goal_id
{
public:
  Init_RobotMotion_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_control_msgs::action::RobotMotion_GetResult_Request goal_id(::robot_control_msgs::action::RobotMotion_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_GetResult_Request>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_GetResult_Request_goal_id();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_GetResult_Response_result
{
public:
  explicit Init_RobotMotion_GetResult_Response_result(::robot_control_msgs::action::RobotMotion_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::action::RobotMotion_GetResult_Response result(::robot_control_msgs::action::RobotMotion_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_GetResult_Response msg_;
};

class Init_RobotMotion_GetResult_Response_status
{
public:
  Init_RobotMotion_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotMotion_GetResult_Response_result status(::robot_control_msgs::action::RobotMotion_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobotMotion_GetResult_Response_result(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_GetResult_Response>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_GetResult_Response_status();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_GetResult_Event_response
{
public:
  explicit Init_RobotMotion_GetResult_Event_response(::robot_control_msgs::action::RobotMotion_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::action::RobotMotion_GetResult_Event response(::robot_control_msgs::action::RobotMotion_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_GetResult_Event msg_;
};

class Init_RobotMotion_GetResult_Event_request
{
public:
  explicit Init_RobotMotion_GetResult_Event_request(::robot_control_msgs::action::RobotMotion_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_RobotMotion_GetResult_Event_response request(::robot_control_msgs::action::RobotMotion_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RobotMotion_GetResult_Event_response(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_GetResult_Event msg_;
};

class Init_RobotMotion_GetResult_Event_info
{
public:
  Init_RobotMotion_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotMotion_GetResult_Event_request info(::robot_control_msgs::action::RobotMotion_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RobotMotion_GetResult_Event_request(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_GetResult_Event>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_GetResult_Event_info();
}

}  // namespace robot_control_msgs


namespace robot_control_msgs
{

namespace action
{

namespace builder
{

class Init_RobotMotion_FeedbackMessage_feedback
{
public:
  explicit Init_RobotMotion_FeedbackMessage_feedback(::robot_control_msgs::action::RobotMotion_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robot_control_msgs::action::RobotMotion_FeedbackMessage feedback(::robot_control_msgs::action::RobotMotion_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_FeedbackMessage msg_;
};

class Init_RobotMotion_FeedbackMessage_goal_id
{
public:
  Init_RobotMotion_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotMotion_FeedbackMessage_feedback goal_id(::robot_control_msgs::action::RobotMotion_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_RobotMotion_FeedbackMessage_feedback(msg_);
  }

private:
  ::robot_control_msgs::action::RobotMotion_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_control_msgs::action::RobotMotion_FeedbackMessage>()
{
  return robot_control_msgs::action::builder::Init_RobotMotion_FeedbackMessage_goal_id();
}

}  // namespace robot_control_msgs

#endif  // ROBOT_CONTROL_MSGS__ACTION__DETAIL__ROBOT_MOTION__BUILDER_HPP_
