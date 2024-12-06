// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice
#include "turtlebot3_msgs/msg/detail/sensor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
turtlebot3_msgs__msg__SensorState__init(turtlebot3_msgs__msg__SensorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    turtlebot3_msgs__msg__SensorState__fini(msg);
    return false;
  }
  // bumper
  // cliff
  // sonar
  // illumination
  // led
  // button
  // torque
  // left_encoder
  // right_encoder
  // battery
  return true;
}

void
turtlebot3_msgs__msg__SensorState__fini(turtlebot3_msgs__msg__SensorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // bumper
  // cliff
  // sonar
  // illumination
  // led
  // button
  // torque
  // left_encoder
  // right_encoder
  // battery
}

bool
turtlebot3_msgs__msg__SensorState__are_equal(const turtlebot3_msgs__msg__SensorState * lhs, const turtlebot3_msgs__msg__SensorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // bumper
  if (lhs->bumper != rhs->bumper) {
    return false;
  }
  // cliff
  if (lhs->cliff != rhs->cliff) {
    return false;
  }
  // sonar
  if (lhs->sonar != rhs->sonar) {
    return false;
  }
  // illumination
  if (lhs->illumination != rhs->illumination) {
    return false;
  }
  // led
  if (lhs->led != rhs->led) {
    return false;
  }
  // button
  if (lhs->button != rhs->button) {
    return false;
  }
  // torque
  if (lhs->torque != rhs->torque) {
    return false;
  }
  // left_encoder
  if (lhs->left_encoder != rhs->left_encoder) {
    return false;
  }
  // right_encoder
  if (lhs->right_encoder != rhs->right_encoder) {
    return false;
  }
  // battery
  if (lhs->battery != rhs->battery) {
    return false;
  }
  return true;
}

bool
turtlebot3_msgs__msg__SensorState__copy(
  const turtlebot3_msgs__msg__SensorState * input,
  turtlebot3_msgs__msg__SensorState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // bumper
  output->bumper = input->bumper;
  // cliff
  output->cliff = input->cliff;
  // sonar
  output->sonar = input->sonar;
  // illumination
  output->illumination = input->illumination;
  // led
  output->led = input->led;
  // button
  output->button = input->button;
  // torque
  output->torque = input->torque;
  // left_encoder
  output->left_encoder = input->left_encoder;
  // right_encoder
  output->right_encoder = input->right_encoder;
  // battery
  output->battery = input->battery;
  return true;
}

turtlebot3_msgs__msg__SensorState *
turtlebot3_msgs__msg__SensorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_msgs__msg__SensorState * msg = (turtlebot3_msgs__msg__SensorState *)allocator.allocate(sizeof(turtlebot3_msgs__msg__SensorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_msgs__msg__SensorState));
  bool success = turtlebot3_msgs__msg__SensorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_msgs__msg__SensorState__destroy(turtlebot3_msgs__msg__SensorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_msgs__msg__SensorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_msgs__msg__SensorState__Sequence__init(turtlebot3_msgs__msg__SensorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_msgs__msg__SensorState * data = NULL;

  if (size) {
    data = (turtlebot3_msgs__msg__SensorState *)allocator.zero_allocate(size, sizeof(turtlebot3_msgs__msg__SensorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_msgs__msg__SensorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_msgs__msg__SensorState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
turtlebot3_msgs__msg__SensorState__Sequence__fini(turtlebot3_msgs__msg__SensorState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      turtlebot3_msgs__msg__SensorState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

turtlebot3_msgs__msg__SensorState__Sequence *
turtlebot3_msgs__msg__SensorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_msgs__msg__SensorState__Sequence * array = (turtlebot3_msgs__msg__SensorState__Sequence *)allocator.allocate(sizeof(turtlebot3_msgs__msg__SensorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_msgs__msg__SensorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_msgs__msg__SensorState__Sequence__destroy(turtlebot3_msgs__msg__SensorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_msgs__msg__SensorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_msgs__msg__SensorState__Sequence__are_equal(const turtlebot3_msgs__msg__SensorState__Sequence * lhs, const turtlebot3_msgs__msg__SensorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_msgs__msg__SensorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_msgs__msg__SensorState__Sequence__copy(
  const turtlebot3_msgs__msg__SensorState__Sequence * input,
  turtlebot3_msgs__msg__SensorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_msgs__msg__SensorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_msgs__msg__SensorState * data =
      (turtlebot3_msgs__msg__SensorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_msgs__msg__SensorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_msgs__msg__SensorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_msgs__msg__SensorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
