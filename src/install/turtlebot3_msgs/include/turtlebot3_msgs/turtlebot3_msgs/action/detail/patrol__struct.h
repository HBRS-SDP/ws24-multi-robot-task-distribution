// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_msgs:action/Patrol.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__STRUCT_H_
#define TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_Goal
{
  float radius;
} turtlebot3_msgs__action__Patrol_Goal;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_Goal.
typedef struct turtlebot3_msgs__action__Patrol_Goal__Sequence
{
  turtlebot3_msgs__action__Patrol_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_Result
{
  bool success;
} turtlebot3_msgs__action__Patrol_Result;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_Result.
typedef struct turtlebot3_msgs__action__Patrol_Result__Sequence
{
  turtlebot3_msgs__action__Patrol_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_Feedback
{
  float left_time;
} turtlebot3_msgs__action__Patrol_Feedback;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_Feedback.
typedef struct turtlebot3_msgs__action__Patrol_Feedback__Sequence
{
  turtlebot3_msgs__action__Patrol_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "turtlebot3_msgs/action/detail/patrol__struct.h"

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  turtlebot3_msgs__action__Patrol_Goal goal;
} turtlebot3_msgs__action__Patrol_SendGoal_Request;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_SendGoal_Request.
typedef struct turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence
{
  turtlebot3_msgs__action__Patrol_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} turtlebot3_msgs__action__Patrol_SendGoal_Response;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_SendGoal_Response.
typedef struct turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence
{
  turtlebot3_msgs__action__Patrol_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} turtlebot3_msgs__action__Patrol_GetResult_Request;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_GetResult_Request.
typedef struct turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence
{
  turtlebot3_msgs__action__Patrol_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_GetResult_Response
{
  int8_t status;
  turtlebot3_msgs__action__Patrol_Result result;
} turtlebot3_msgs__action__Patrol_GetResult_Response;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_GetResult_Response.
typedef struct turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence
{
  turtlebot3_msgs__action__Patrol_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.h"

/// Struct defined in action/Patrol in the package turtlebot3_msgs.
typedef struct turtlebot3_msgs__action__Patrol_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  turtlebot3_msgs__action__Patrol_Feedback feedback;
} turtlebot3_msgs__action__Patrol_FeedbackMessage;

// Struct for a sequence of turtlebot3_msgs__action__Patrol_FeedbackMessage.
typedef struct turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence
{
  turtlebot3_msgs__action__Patrol_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__action__Patrol_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__STRUCT_H_
