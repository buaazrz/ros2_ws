// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_control_msgs:msg/VectorData.idl
// generated code does not contain a copyright notice
#include "robot_control_msgs/msg/detail/vector_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_control_msgs__msg__VectorData__init(robot_control_msgs__msg__VectorData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robot_control_msgs__msg__VectorData__fini(msg);
    return false;
  }
  // timestamp
  // data
  if (!rosidl_runtime_c__double__Sequence__init(&msg->data, 0)) {
    robot_control_msgs__msg__VectorData__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    robot_control_msgs__msg__VectorData__fini(msg);
    return false;
  }
  return true;
}

void
robot_control_msgs__msg__VectorData__fini(robot_control_msgs__msg__VectorData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // timestamp
  // data
  rosidl_runtime_c__double__Sequence__fini(&msg->data);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
}

bool
robot_control_msgs__msg__VectorData__are_equal(const robot_control_msgs__msg__VectorData * lhs, const robot_control_msgs__msg__VectorData * rhs)
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
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  return true;
}

bool
robot_control_msgs__msg__VectorData__copy(
  const robot_control_msgs__msg__VectorData * input,
  robot_control_msgs__msg__VectorData * output)
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
  // timestamp
  output->timestamp = input->timestamp;
  // data
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  return true;
}

robot_control_msgs__msg__VectorData *
robot_control_msgs__msg__VectorData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__msg__VectorData * msg = (robot_control_msgs__msg__VectorData *)allocator.allocate(sizeof(robot_control_msgs__msg__VectorData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control_msgs__msg__VectorData));
  bool success = robot_control_msgs__msg__VectorData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control_msgs__msg__VectorData__destroy(robot_control_msgs__msg__VectorData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control_msgs__msg__VectorData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control_msgs__msg__VectorData__Sequence__init(robot_control_msgs__msg__VectorData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__msg__VectorData * data = NULL;

  if (size) {
    data = (robot_control_msgs__msg__VectorData *)allocator.zero_allocate(size, sizeof(robot_control_msgs__msg__VectorData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control_msgs__msg__VectorData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control_msgs__msg__VectorData__fini(&data[i - 1]);
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
robot_control_msgs__msg__VectorData__Sequence__fini(robot_control_msgs__msg__VectorData__Sequence * array)
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
      robot_control_msgs__msg__VectorData__fini(&array->data[i]);
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

robot_control_msgs__msg__VectorData__Sequence *
robot_control_msgs__msg__VectorData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__msg__VectorData__Sequence * array = (robot_control_msgs__msg__VectorData__Sequence *)allocator.allocate(sizeof(robot_control_msgs__msg__VectorData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control_msgs__msg__VectorData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control_msgs__msg__VectorData__Sequence__destroy(robot_control_msgs__msg__VectorData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control_msgs__msg__VectorData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control_msgs__msg__VectorData__Sequence__are_equal(const robot_control_msgs__msg__VectorData__Sequence * lhs, const robot_control_msgs__msg__VectorData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control_msgs__msg__VectorData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control_msgs__msg__VectorData__Sequence__copy(
  const robot_control_msgs__msg__VectorData__Sequence * input,
  robot_control_msgs__msg__VectorData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_control_msgs__msg__VectorData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control_msgs__msg__VectorData * data =
      (robot_control_msgs__msg__VectorData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control_msgs__msg__VectorData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control_msgs__msg__VectorData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control_msgs__msg__VectorData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
