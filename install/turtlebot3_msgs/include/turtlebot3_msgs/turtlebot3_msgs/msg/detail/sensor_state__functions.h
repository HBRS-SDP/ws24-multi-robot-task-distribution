// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__FUNCTIONS_H_
#define TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "turtlebot3_msgs/msg/detail/sensor_state__struct.h"

/// Initialize msg/SensorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_msgs__msg__SensorState
 * )) before or use
 * turtlebot3_msgs__msg__SensorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__SensorState__init(turtlebot3_msgs__msg__SensorState * msg);

/// Finalize msg/SensorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__SensorState__fini(turtlebot3_msgs__msg__SensorState * msg);

/// Create msg/SensorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_msgs__msg__SensorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__msg__SensorState *
turtlebot3_msgs__msg__SensorState__create();

/// Destroy msg/SensorState message.
/**
 * It calls
 * turtlebot3_msgs__msg__SensorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__SensorState__destroy(turtlebot3_msgs__msg__SensorState * msg);

/// Check for msg/SensorState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__SensorState__are_equal(const turtlebot3_msgs__msg__SensorState * lhs, const turtlebot3_msgs__msg__SensorState * rhs);

/// Copy a msg/SensorState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__SensorState__copy(
  const turtlebot3_msgs__msg__SensorState * input,
  turtlebot3_msgs__msg__SensorState * output);

/// Initialize array of msg/SensorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_msgs__msg__SensorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__SensorState__Sequence__init(turtlebot3_msgs__msg__SensorState__Sequence * array, size_t size);

/// Finalize array of msg/SensorState messages.
/**
 * It calls
 * turtlebot3_msgs__msg__SensorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__SensorState__Sequence__fini(turtlebot3_msgs__msg__SensorState__Sequence * array);

/// Create array of msg/SensorState messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_msgs__msg__SensorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__msg__SensorState__Sequence *
turtlebot3_msgs__msg__SensorState__Sequence__create(size_t size);

/// Destroy array of msg/SensorState messages.
/**
 * It calls
 * turtlebot3_msgs__msg__SensorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__msg__SensorState__Sequence__destroy(turtlebot3_msgs__msg__SensorState__Sequence * array);

/// Check for msg/SensorState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__SensorState__Sequence__are_equal(const turtlebot3_msgs__msg__SensorState__Sequence * lhs, const turtlebot3_msgs__msg__SensorState__Sequence * rhs);

/// Copy an array of msg/SensorState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__msg__SensorState__Sequence__copy(
  const turtlebot3_msgs__msg__SensorState__Sequence * input,
  turtlebot3_msgs__msg__SensorState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__SENSOR_STATE__FUNCTIONS_H_
