// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:action/Patrol.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__BUILDER_HPP_
#define TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/action/detail/patrol__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_Goal_radius
{
public:
  Init_Patrol_Goal_radius()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_msgs::action::Patrol_Goal radius(::turtlebot3_msgs::action::Patrol_Goal::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_Goal>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_Goal_radius();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_Result_success
{
public:
  Init_Patrol_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_msgs::action::Patrol_Result success(::turtlebot3_msgs::action::Patrol_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_Result>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_Result_success();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_Feedback_left_time
{
public:
  Init_Patrol_Feedback_left_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_msgs::action::Patrol_Feedback left_time(::turtlebot3_msgs::action::Patrol_Feedback::_left_time_type arg)
  {
    msg_.left_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_Feedback>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_Feedback_left_time();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_SendGoal_Request_goal
{
public:
  explicit Init_Patrol_SendGoal_Request_goal(::turtlebot3_msgs::action::Patrol_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::action::Patrol_SendGoal_Request goal(::turtlebot3_msgs::action::Patrol_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_SendGoal_Request msg_;
};

class Init_Patrol_SendGoal_Request_goal_id
{
public:
  Init_Patrol_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_SendGoal_Request_goal goal_id(::turtlebot3_msgs::action::Patrol_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Patrol_SendGoal_Request_goal(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_SendGoal_Request>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_SendGoal_Request_goal_id();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_SendGoal_Response_stamp
{
public:
  explicit Init_Patrol_SendGoal_Response_stamp(::turtlebot3_msgs::action::Patrol_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::action::Patrol_SendGoal_Response stamp(::turtlebot3_msgs::action::Patrol_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_SendGoal_Response msg_;
};

class Init_Patrol_SendGoal_Response_accepted
{
public:
  Init_Patrol_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_SendGoal_Response_stamp accepted(::turtlebot3_msgs::action::Patrol_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Patrol_SendGoal_Response_stamp(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_SendGoal_Response>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_SendGoal_Response_accepted();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_GetResult_Request_goal_id
{
public:
  Init_Patrol_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_msgs::action::Patrol_GetResult_Request goal_id(::turtlebot3_msgs::action::Patrol_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_GetResult_Request>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_GetResult_Request_goal_id();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_GetResult_Response_result
{
public:
  explicit Init_Patrol_GetResult_Response_result(::turtlebot3_msgs::action::Patrol_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::action::Patrol_GetResult_Response result(::turtlebot3_msgs::action::Patrol_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_GetResult_Response msg_;
};

class Init_Patrol_GetResult_Response_status
{
public:
  Init_Patrol_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_GetResult_Response_result status(::turtlebot3_msgs::action::Patrol_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Patrol_GetResult_Response_result(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_GetResult_Response>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_GetResult_Response_status();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace action
{

namespace builder
{

class Init_Patrol_FeedbackMessage_feedback
{
public:
  explicit Init_Patrol_FeedbackMessage_feedback(::turtlebot3_msgs::action::Patrol_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::action::Patrol_FeedbackMessage feedback(::turtlebot3_msgs::action::Patrol_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_FeedbackMessage msg_;
};

class Init_Patrol_FeedbackMessage_goal_id
{
public:
  Init_Patrol_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Patrol_FeedbackMessage_feedback goal_id(::turtlebot3_msgs::action::Patrol_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Patrol_FeedbackMessage_feedback(msg_);
  }

private:
  ::turtlebot3_msgs::action::Patrol_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::action::Patrol_FeedbackMessage>()
{
  return turtlebot3_msgs::action::builder::Init_Patrol_FeedbackMessage_goal_id();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__BUILDER_HPP_
