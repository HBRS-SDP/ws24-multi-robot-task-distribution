// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_msgs:msg/Sound.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__STRUCT_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OFF'.
enum
{
  turtlebot3_msgs__msg__Sound__OFF = 0
};

/// Constant 'ON'.
enum
{
  turtlebot3_msgs__msg__Sound__ON = 1
};

/// Constant 'LOW_BATTERY'.
enum
{
  turtlebot3_msgs__msg__Sound__LOW_BATTERY = 2
};

/// Constant 'ERROR'.
enum
{
  turtlebot3_msgs__msg__Sound__ERROR = 3
};

/// Constant 'BUTTON1'.
enum
{
  turtlebot3_msgs__msg__Sound__BUTTON1 = 4
};

/// Constant 'BUTTON2'.
enum
{
  turtlebot3_msgs__msg__Sound__BUTTON2 = 5
};

/// Struct defined in msg/Sound in the package turtlebot3_msgs.
/**
  * CONSTANTS
 */
typedef struct turtlebot3_msgs__msg__Sound
{
  /// Messages
  uint8_t value;
} turtlebot3_msgs__msg__Sound;

// Struct for a sequence of turtlebot3_msgs__msg__Sound.
typedef struct turtlebot3_msgs__msg__Sound__Sequence
{
  turtlebot3_msgs__msg__Sound * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_msgs__msg__Sound__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SOUND__STRUCT_H_
