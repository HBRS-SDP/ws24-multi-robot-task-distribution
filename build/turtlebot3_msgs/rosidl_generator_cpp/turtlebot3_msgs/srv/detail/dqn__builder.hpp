// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:srv/Dqn.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__DQN__BUILDER_HPP_
#define TURTLEBOT3_MSGS__SRV__DETAIL__DQN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/srv/detail/dqn__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace srv
{

namespace builder
{

class Init_Dqn_Request_init
{
public:
  explicit Init_Dqn_Request_init(::turtlebot3_msgs::srv::Dqn_Request & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::srv::Dqn_Request init(::turtlebot3_msgs::srv::Dqn_Request::_init_type arg)
  {
    msg_.init = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Dqn_Request msg_;
};

class Init_Dqn_Request_action
{
public:
  Init_Dqn_Request_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dqn_Request_init action(::turtlebot3_msgs::srv::Dqn_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_Dqn_Request_init(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Dqn_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::srv::Dqn_Request>()
{
  return turtlebot3_msgs::srv::builder::Init_Dqn_Request_action();
}

}  // namespace turtlebot3_msgs


namespace turtlebot3_msgs
{

namespace srv
{

namespace builder
{

class Init_Dqn_Response_done
{
public:
  explicit Init_Dqn_Response_done(::turtlebot3_msgs::srv::Dqn_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::srv::Dqn_Response done(::turtlebot3_msgs::srv::Dqn_Response::_done_type arg)
  {
    msg_.done = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Dqn_Response msg_;
};

class Init_Dqn_Response_reward
{
public:
  explicit Init_Dqn_Response_reward(::turtlebot3_msgs::srv::Dqn_Response & msg)
  : msg_(msg)
  {}
  Init_Dqn_Response_done reward(::turtlebot3_msgs::srv::Dqn_Response::_reward_type arg)
  {
    msg_.reward = std::move(arg);
    return Init_Dqn_Response_done(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Dqn_Response msg_;
};

class Init_Dqn_Response_state
{
public:
  Init_Dqn_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dqn_Response_reward state(::turtlebot3_msgs::srv::Dqn_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_Dqn_Response_reward(msg_);
  }

private:
  ::turtlebot3_msgs::srv::Dqn_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::srv::Dqn_Response>()
{
  return turtlebot3_msgs::srv::builder::Init_Dqn_Response_state();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__DQN__BUILDER_HPP_
