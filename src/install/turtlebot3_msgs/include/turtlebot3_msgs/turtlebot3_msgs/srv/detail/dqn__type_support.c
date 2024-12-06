// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from turtlebot3_msgs:srv/Dqn.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "turtlebot3_msgs/srv/detail/dqn__rosidl_typesupport_introspection_c.h"
#include "turtlebot3_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "turtlebot3_msgs/srv/detail/dqn__functions.h"
#include "turtlebot3_msgs/srv/detail/dqn__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot3_msgs__srv__Dqn_Request__init(message_memory);
}

void turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_fini_function(void * message_memory)
{
  turtlebot3_msgs__srv__Dqn_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_member_array[2] = {
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_msgs__srv__Dqn_Request, action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "init",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_msgs__srv__Dqn_Request, init),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_members = {
  "turtlebot3_msgs__srv",  // message namespace
  "Dqn_Request",  // message name
  2,  // number of fields
  sizeof(turtlebot3_msgs__srv__Dqn_Request),
  turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_member_array,  // message members
  turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_type_support_handle = {
  0,
  &turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn_Request)() {
  if (!turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_type_support_handle.typesupport_identifier) {
    turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot3_msgs__srv__Dqn_Request__rosidl_typesupport_introspection_c__Dqn_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "turtlebot3_msgs/srv/detail/dqn__rosidl_typesupport_introspection_c.h"
// already included above
// #include "turtlebot3_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "turtlebot3_msgs/srv/detail/dqn__functions.h"
// already included above
// #include "turtlebot3_msgs/srv/detail/dqn__struct.h"


// Include directives for member types
// Member `state`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  turtlebot3_msgs__srv__Dqn_Response__init(message_memory);
}

void turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_fini_function(void * message_memory)
{
  turtlebot3_msgs__srv__Dqn_Response__fini(message_memory);
}

size_t turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__size_function__Dqn_Response__state(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__get_const_function__Dqn_Response__state(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__get_function__Dqn_Response__state(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__fetch_function__Dqn_Response__state(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__get_const_function__Dqn_Response__state(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__assign_function__Dqn_Response__state(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__get_function__Dqn_Response__state(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__resize_function__Dqn_Response__state(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_member_array[3] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_msgs__srv__Dqn_Response, state),  // bytes offset in struct
    NULL,  // default value
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__size_function__Dqn_Response__state,  // size() function pointer
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__get_const_function__Dqn_Response__state,  // get_const(index) function pointer
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__get_function__Dqn_Response__state,  // get(index) function pointer
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__fetch_function__Dqn_Response__state,  // fetch(index, &value) function pointer
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__assign_function__Dqn_Response__state,  // assign(index, value) function pointer
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__resize_function__Dqn_Response__state  // resize(index) function pointer
  },
  {
    "reward",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_msgs__srv__Dqn_Response, reward),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "done",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtlebot3_msgs__srv__Dqn_Response, done),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_members = {
  "turtlebot3_msgs__srv",  // message namespace
  "Dqn_Response",  // message name
  3,  // number of fields
  sizeof(turtlebot3_msgs__srv__Dqn_Response),
  turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_member_array,  // message members
  turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_type_support_handle = {
  0,
  &turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn_Response)() {
  if (!turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_type_support_handle.typesupport_identifier) {
    turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &turtlebot3_msgs__srv__Dqn_Response__rosidl_typesupport_introspection_c__Dqn_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "turtlebot3_msgs/srv/detail/dqn__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_members = {
  "turtlebot3_msgs__srv",  // service namespace
  "Dqn",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_Request_message_type_support_handle,
  NULL  // response message
  // turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_Response_message_type_support_handle
};

static rosidl_service_type_support_t turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_type_support_handle = {
  0,
  &turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_turtlebot3_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn)() {
  if (!turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_type_support_handle.typesupport_identifier) {
    turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, srv, Dqn_Response)()->data;
  }

  return &turtlebot3_msgs__srv__detail__dqn__rosidl_typesupport_introspection_c__Dqn_service_type_support_handle;
}
