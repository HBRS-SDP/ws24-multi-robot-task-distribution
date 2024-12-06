// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:srv/Sound.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__BUILDER_HPP_
#define TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/srv/detail/sound__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace srv
{

namespace builder
{

class Init_Sound_Request_value
{
public:
  Init_Sound_Request_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_msgs::srv::Sound_Request value(::turtlebot3_msgs::srv::Sound_Request::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Sound_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::srv::Sound_Request>()
{
  return turtlebot3_msgs::srv::builder::Init_Sound_Request_value();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace srv
{

namespace builder
{

class Init_Sound_Response_message
{
public:
  explicit Init_Sound_Response_message(::turtlebot3_msgs::srv::Sound_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::srv::Sound_Response message(::turtlebot3_msgs::srv::Sound_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Sound_Response msg_;
};

class Init_Sound_Response_success
{
public:
  Init_Sound_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sound_Response_message success(::turtlebot3_msgs::srv::Sound_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Sound_Response_message(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Sound_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::srv::Sound_Response>()
{
  return turtlebot3_msgs::srv::builder::Init_Sound_Response_success();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__BUILDER_HPP_
