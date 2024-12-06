// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_msgs:srv/Dqn.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__DQN__TRAITS_HPP_
#define TURTLEBOT3_MSGS__SRV__DETAIL__DQN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_msgs/srv/detail/dqn__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Dqn_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: init
  {
    out << "init: ";
    rosidl_generator_traits::value_to_yaml(msg.init, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Dqn_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: init
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "init: ";
    rosidl_generator_traits::value_to_yaml(msg.init, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Dqn_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_msgs::srv::Dqn_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_msgs::srv::Dqn_Request & msg)
{
  return turtlebot3_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_msgs::srv::Dqn_Request>()
{
  return "turtlebot3_msgs::srv::Dqn_Request";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Dqn_Request>()
{
  return "turtlebot3_msgs/srv/Dqn_Request";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Dqn_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Dqn_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_msgs::srv::Dqn_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Dqn_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    if (msg.state.size() == 0) {
      out << "state: []";
    } else {
      out << "state: [";
      size_t pending_items = msg.state.size();
      for (auto item : msg.state) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: reward
  {
    out << "reward: ";
    rosidl_generator_traits::value_to_yaml(msg.reward, out);
    out << ", ";
  }

  // member: done
  {
    out << "done: ";
    rosidl_generator_traits::value_to_yaml(msg.done, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Dqn_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.state.size() == 0) {
      out << "state: []\n";
    } else {
      out << "state:\n";
      for (auto item : msg.state) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: reward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reward: ";
    rosidl_generator_traits::value_to_yaml(msg.reward, out);
    out << "\n";
  }

  // member: done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "done: ";
    rosidl_generator_traits::value_to_yaml(msg.done, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Dqn_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_msgs

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_msgs::srv::Dqn_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_msgs::srv::Dqn_Response & msg)
{
  return turtlebot3_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_msgs::srv::Dqn_Response>()
{
  return "turtlebot3_msgs::srv::Dqn_Response";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Dqn_Response>()
{
  return "turtlebot3_msgs/srv/Dqn_Response";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Dqn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Dqn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_msgs::srv::Dqn_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_msgs::srv::Dqn>()
{
  return "turtlebot3_msgs::srv::Dqn";
}

template<>
inline const char * name<turtlebot3_msgs::srv::Dqn>()
{
  return "turtlebot3_msgs/srv/Dqn";
}

template<>
struct has_fixed_size<turtlebot3_msgs::srv::Dqn>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_msgs::srv::Dqn_Request>::value &&
    has_fixed_size<turtlebot3_msgs::srv::Dqn_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_msgs::srv::Dqn>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_msgs::srv::Dqn_Request>::value &&
    has_bounded_size<turtlebot3_msgs::srv::Dqn_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_msgs::srv::Dqn>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_msgs::srv::Dqn_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_msgs::srv::Dqn_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__DQN__TRAITS_HPP_
