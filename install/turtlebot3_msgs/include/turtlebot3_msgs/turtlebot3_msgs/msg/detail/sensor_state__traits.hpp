// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__TRAITS_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_msgs/msg/detail/sensor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace turtlebot3_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: bumper
  {
    out << "bumper: ";
    rosidl_generator_traits::value_to_yaml(msg.bumper, out);
    out << ", ";
  }

  // member: cliff
  {
    out << "cliff: ";
    rosidl_generator_traits::value_to_yaml(msg.cliff, out);
    out << ", ";
  }

  // member: sonar
  {
    out << "sonar: ";
    rosidl_generator_traits::value_to_yaml(msg.sonar, out);
    out << ", ";
  }

  // member: illumination
  {
    out << "illumination: ";
    rosidl_generator_traits::value_to_yaml(msg.illumination, out);
    out << ", ";
  }

  // member: led
  {
    out << "led: ";
    rosidl_generator_traits::value_to_yaml(msg.led, out);
    out << ", ";
  }

  // member: button
  {
    out << "button: ";
    rosidl_generator_traits::value_to_yaml(msg.button, out);
    out << ", ";
  }

  // member: torque
  {
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
    out << ", ";
  }

  // member: left_encoder
  {
    out << "left_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_encoder, out);
    out << ", ";
  }

  // member: right_encoder
  {
    out << "right_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_encoder, out);
    out << ", ";
  }

  // member: battery
  {
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorState & msg,
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

  // member: bumper
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bumper: ";
    rosidl_generator_traits::value_to_yaml(msg.bumper, out);
    out << "\n";
  }

  // member: cliff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cliff: ";
    rosidl_generator_traits::value_to_yaml(msg.cliff, out);
    out << "\n";
  }

  // member: sonar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sonar: ";
    rosidl_generator_traits::value_to_yaml(msg.sonar, out);
    out << "\n";
  }

  // member: illumination
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "illumination: ";
    rosidl_generator_traits::value_to_yaml(msg.illumination, out);
    out << "\n";
  }

  // member: led
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "led: ";
    rosidl_generator_traits::value_to_yaml(msg.led, out);
    out << "\n";
  }

  // member: button
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button: ";
    rosidl_generator_traits::value_to_yaml(msg.button, out);
    out << "\n";
  }

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
    out << "\n";
  }

  // member: left_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_encoder, out);
    out << "\n";
  }

  // member: right_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_encoder, out);
    out << "\n";
  }

  // member: battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorState & msg, bool use_flow_style = false)
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

}  // namespace turtlebot3_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_msgs::msg::SensorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_msgs::msg::SensorState & msg)
{
  return turtlebot3_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_msgs::msg::SensorState>()
{
  return "turtlebot3_msgs::msg::SensorState";
}

template<>
inline const char * name<turtlebot3_msgs::msg::SensorState>()
{
  return "turtlebot3_msgs/msg/SensorState";
}

template<>
struct has_fixed_size<turtlebot3_msgs::msg::SensorState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<turtlebot3_msgs::msg::SensorState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<turtlebot3_msgs::msg::SensorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__TRAITS_HPP_
