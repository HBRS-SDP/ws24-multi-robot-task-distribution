// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__BUILDER_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_msgs/msg/detail/sensor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_msgs
{

namespace msg
{

namespace builder
{

class Init_SensorState_battery
{
public:
  explicit Init_SensorState_battery(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  ::turtlebot3_msgs::msg::SensorState battery(::turtlebot3_msgs::msg::SensorState::_battery_type arg)
  {
    msg_.battery = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_right_encoder
{
public:
  explicit Init_SensorState_right_encoder(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_battery right_encoder(::turtlebot3_msgs::msg::SensorState::_right_encoder_type arg)
  {
    msg_.right_encoder = std::move(arg);
    return Init_SensorState_battery(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_left_encoder
{
public:
  explicit Init_SensorState_left_encoder(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_right_encoder left_encoder(::turtlebot3_msgs::msg::SensorState::_left_encoder_type arg)
  {
    msg_.left_encoder = std::move(arg);
    return Init_SensorState_right_encoder(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_torque
{
public:
  explicit Init_SensorState_torque(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_left_encoder torque(::turtlebot3_msgs::msg::SensorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_SensorState_left_encoder(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_button
{
public:
  explicit Init_SensorState_button(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_torque button(::turtlebot3_msgs::msg::SensorState::_button_type arg)
  {
    msg_.button = std::move(arg);
    return Init_SensorState_torque(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_led
{
public:
  explicit Init_SensorState_led(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_button led(::turtlebot3_msgs::msg::SensorState::_led_type arg)
  {
    msg_.led = std::move(arg);
    return Init_SensorState_button(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_illumination
{
public:
  explicit Init_SensorState_illumination(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_led illumination(::turtlebot3_msgs::msg::SensorState::_illumination_type arg)
  {
    msg_.illumination = std::move(arg);
    return Init_SensorState_led(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_sonar
{
public:
  explicit Init_SensorState_sonar(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_illumination sonar(::turtlebot3_msgs::msg::SensorState::_sonar_type arg)
  {
    msg_.sonar = std::move(arg);
    return Init_SensorState_illumination(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_cliff
{
public:
  explicit Init_SensorState_cliff(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_sonar cliff(::turtlebot3_msgs::msg::SensorState::_cliff_type arg)
  {
    msg_.cliff = std::move(arg);
    return Init_SensorState_sonar(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_bumper
{
public:
  explicit Init_SensorState_bumper(::turtlebot3_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_cliff bumper(::turtlebot3_msgs::msg::SensorState::_bumper_type arg)
  {
    msg_.bumper = std::move(arg);
    return Init_SensorState_cliff(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

class Init_SensorState_header
{
public:
  Init_SensorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorState_bumper header(::turtlebot3_msgs::msg::SensorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorState_bumper(msg_);
  }

private:
  ::turtlebot3_msgs::msg::SensorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_msgs::msg::SensorState>()
{
  return turtlebot3_msgs::msg::builder::Init_SensorState_header();
}

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__BUILDER_HPP_
