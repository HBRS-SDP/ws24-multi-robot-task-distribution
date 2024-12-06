// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from turtlebot3_msgs:action/Patrol.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "turtlebot3_msgs/action/detail/patrol__struct.h"
#include "turtlebot3_msgs/action/detail/patrol__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_Goal_type_support_ids_t;

static const _Patrol_Goal_type_support_ids_t _Patrol_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_Goal_type_support_symbol_names_t _Patrol_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_Goal)),
  }
};

typedef struct _Patrol_Goal_type_support_data_t
{
  void * data[2];
} _Patrol_Goal_type_support_data_t;

static _Patrol_Goal_type_support_data_t _Patrol_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_Goal_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_Goal)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_Result_type_support_ids_t;

static const _Patrol_Result_type_support_ids_t _Patrol_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_Result_type_support_symbol_names_t _Patrol_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_Result)),
  }
};

typedef struct _Patrol_Result_type_support_data_t
{
  void * data[2];
} _Patrol_Result_type_support_data_t;

static _Patrol_Result_type_support_data_t _Patrol_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_Result_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_Result)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_Feedback_type_support_ids_t;

static const _Patrol_Feedback_type_support_ids_t _Patrol_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_Feedback_type_support_symbol_names_t _Patrol_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_Feedback)),
  }
};

typedef struct _Patrol_Feedback_type_support_data_t
{
  void * data[2];
} _Patrol_Feedback_type_support_data_t;

static _Patrol_Feedback_type_support_data_t _Patrol_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_Feedback_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_Feedback)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_SendGoal_Request_type_support_ids_t;

static const _Patrol_SendGoal_Request_type_support_ids_t _Patrol_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_SendGoal_Request_type_support_symbol_names_t _Patrol_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_SendGoal_Request)),
  }
};

typedef struct _Patrol_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Patrol_SendGoal_Request_type_support_data_t;

static _Patrol_SendGoal_Request_type_support_data_t _Patrol_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_SendGoal_Request_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_SendGoal_Request)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_SendGoal_Response_type_support_ids_t;

static const _Patrol_SendGoal_Response_type_support_ids_t _Patrol_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_SendGoal_Response_type_support_symbol_names_t _Patrol_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_SendGoal_Response)),
  }
};

typedef struct _Patrol_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Patrol_SendGoal_Response_type_support_data_t;

static _Patrol_SendGoal_Response_type_support_data_t _Patrol_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_SendGoal_Response_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_SendGoal_Response)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_SendGoal_type_support_ids_t;

static const _Patrol_SendGoal_type_support_ids_t _Patrol_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_SendGoal_type_support_symbol_names_t _Patrol_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_SendGoal)),
  }
};

typedef struct _Patrol_SendGoal_type_support_data_t
{
  void * data[2];
} _Patrol_SendGoal_type_support_data_t;

static _Patrol_SendGoal_type_support_data_t _Patrol_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_SendGoal_service_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Patrol_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Patrol_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Patrol_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_SendGoal)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_GetResult_Request_type_support_ids_t;

static const _Patrol_GetResult_Request_type_support_ids_t _Patrol_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_GetResult_Request_type_support_symbol_names_t _Patrol_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_GetResult_Request)),
  }
};

typedef struct _Patrol_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Patrol_GetResult_Request_type_support_data_t;

static _Patrol_GetResult_Request_type_support_data_t _Patrol_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_GetResult_Request_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_GetResult_Request)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_GetResult_Response_type_support_ids_t;

static const _Patrol_GetResult_Response_type_support_ids_t _Patrol_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_GetResult_Response_type_support_symbol_names_t _Patrol_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_GetResult_Response)),
  }
};

typedef struct _Patrol_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Patrol_GetResult_Response_type_support_data_t;

static _Patrol_GetResult_Response_type_support_data_t _Patrol_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_GetResult_Response_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_GetResult_Response)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_GetResult_type_support_ids_t;

static const _Patrol_GetResult_type_support_ids_t _Patrol_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_GetResult_type_support_symbol_names_t _Patrol_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_GetResult)),
  }
};

typedef struct _Patrol_GetResult_type_support_data_t
{
  void * data[2];
} _Patrol_GetResult_type_support_data_t;

static _Patrol_GetResult_type_support_data_t _Patrol_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_GetResult_service_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Patrol_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Patrol_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Patrol_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_GetResult)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlebot3_msgs
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Patrol_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Patrol_FeedbackMessage_type_support_ids_t;

static const _Patrol_FeedbackMessage_type_support_ids_t _Patrol_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Patrol_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Patrol_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Patrol_FeedbackMessage_type_support_symbol_names_t _Patrol_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtlebot3_msgs, action, Patrol_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, turtlebot3_msgs, action, Patrol_FeedbackMessage)),
  }
};

typedef struct _Patrol_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Patrol_FeedbackMessage_type_support_data_t;

static _Patrol_FeedbackMessage_type_support_data_t _Patrol_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Patrol_FeedbackMessage_message_typesupport_map = {
  2,
  "turtlebot3_msgs",
  &_Patrol_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Patrol_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Patrol_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Patrol_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Patrol_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace turtlebot3_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_FeedbackMessage)() {
  return &::turtlebot3_msgs::action::rosidl_typesupport_c::Patrol_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "turtlebot3_msgs/action/patrol.h"
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__type_support.h"

static rosidl_action_type_support_t _turtlebot3_msgs__action__Patrol__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, turtlebot3_msgs, action, Patrol)()
{
  // Thread-safe by always writing the same values to the static struct
  _turtlebot3_msgs__action__Patrol__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_SendGoal)();
  _turtlebot3_msgs__action__Patrol__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_GetResult)();
  _turtlebot3_msgs__action__Patrol__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _turtlebot3_msgs__action__Patrol__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, turtlebot3_msgs, action, Patrol_FeedbackMessage)();
  _turtlebot3_msgs__action__Patrol__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_turtlebot3_msgs__action__Patrol__typesupport_c;
}

#ifdef __cplusplus
}
#endif
