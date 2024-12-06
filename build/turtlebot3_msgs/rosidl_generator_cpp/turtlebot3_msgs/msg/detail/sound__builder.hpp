// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:msg/Sound.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__BUILDER_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/msg/detail/sound__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace msg
{

namespace builder
{

class Init_Sound_value
{
public:
  Init_Sound_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_msgs::msg::Sound value(::turtlebot3_msgs::msg::Sound::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::msg::Sound msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::msg::Sound>()
{
  return turtlebot3_msgs::msg::builder::Init_Sound_value();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__BUILDER_HPP_
