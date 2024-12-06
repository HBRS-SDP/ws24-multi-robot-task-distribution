// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from turtlebot3_msgs:msg/Sound.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "turtlebot3_msgs/msg/detail/sound__rosidl_typesupport_introspection_c.h"
#include "turtlebot3_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "turtlebot3_msgs/msg/detail/sound__functions.h"
#include "turtlebot3_msgs/msg/detail/sound__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot3_msgs__msg__Sound__init(message_memory);
}

void turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_fini_function(void * message_memory)
{
  turtlebot3_msgs__msg__Sound__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_member_array[1] = {
  {
    "value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_msgs__msg__Sound, value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_members = {
  "turtlebot3_msgs__msg",  // message namespace
  "Sound",  // message name
  1,  // number of fields
  sizeof(turtlebot3_msgs__msg__Sound),
  turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_member_array,  // message members
  turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_type_support_handle = {
  0,
  &turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, msg, Sound)() {
  if (!turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_type_support_handle.typesupport_identifier) {
    turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot3_msgs__msg__Sound__rosidl_typesupport_introspection_c__Sound_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
