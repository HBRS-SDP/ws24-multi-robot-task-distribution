// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from turtlebot3_msgs:msg/Sound.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "turtlebot3_msgs/msg/detail/sound__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _Sound_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Sound_type_support_ids_t;

static const _Sound_type_support_ids_t _Sound_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Sound_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Sound_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Sound_type_support_symbol_names_t _Sound_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlebot3_msgs, msg, Sound)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, turtlebot3_msgs, msg, Sound)),
  }
};

typedef struct _Sound_type_support_data_t
{
  void * data[2];
} _Sound_type_support_data_t;

static _Sound_type_support_data_t _Sound_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Sound_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Sound_message_typesupport_ids.typesupport_identifier[0],
  &_Sound_message_typesupport_symbol_names.symbol_name[0],
  &_Sound_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Sound_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Sound_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace turtlebot3_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<turtlebot3_msgs::msg::Sound>()
{
  return &::turtlebot3_msgs::msg::rosidl_typesupport_cpp::Sound_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, turtlebot3_msgs, msg, Sound)() {
  return get_message_type_support_handle<turtlebot3_msgs::msg::Sound>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
