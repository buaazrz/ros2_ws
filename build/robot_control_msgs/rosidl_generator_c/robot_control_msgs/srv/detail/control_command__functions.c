// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_control_msgs:srv/ControlCommand.idl
// generated code does not contain a copyright notice
#include "robot_control_msgs/srv/detail/control_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `cmd_name`
// Member `cmd_params`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_control_msgs__srv__ControlCommand_Request__init(robot_control_msgs__srv__ControlCommand_Request * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_name
  if (!rosidl_runtime_c__String__init(&msg->cmd_name)) {
    robot_control_msgs__srv__ControlCommand_Request__fini(msg);
    return false;
  }
  // cmd_params
  if (!rosidl_runtime_c__String__init(&msg->cmd_params)) {
    robot_control_msgs__srv__ControlCommand_Request__fini(msg);
    return false;
  }
  return true;
}

void
robot_control_msgs__srv__ControlCommand_Request__fini(robot_control_msgs__srv__ControlCommand_Request * msg)
{
  if (!msg) {
    return;
  }
  // cmd_name
  rosidl_runtime_c__String__fini(&msg->cmd_name);
  // cmd_params
  rosidl_runtime_c__String__fini(&msg->cmd_params);
}

bool
robot_control_msgs__srv__ControlCommand_Request__are_equal(const robot_control_msgs__srv__ControlCommand_Request * lhs, const robot_control_msgs__srv__ControlCommand_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cmd_name), &(rhs->cmd_name)))
  {
    return false;
  }
  // cmd_params
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cmd_params), &(rhs->cmd_params)))
  {
    return false;
  }
  return true;
}

bool
robot_control_msgs__srv__ControlCommand_Request__copy(
  const robot_control_msgs__srv__ControlCommand_Request * input,
  robot_control_msgs__srv__ControlCommand_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_name
  if (!rosidl_runtime_c__String__copy(
      &(input->cmd_name), &(output->cmd_name)))
  {
    return false;
  }
  // cmd_params
  if (!rosidl_runtime_c__String__copy(
      &(input->cmd_params), &(output->cmd_params)))
  {
    return false;
  }
  return true;
}

robot_control_msgs__srv__ControlCommand_Request *
robot_control_msgs__srv__ControlCommand_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Request * msg = (robot_control_msgs__srv__ControlCommand_Request *)allocator.allocate(sizeof(robot_control_msgs__srv__ControlCommand_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control_msgs__srv__ControlCommand_Request));
  bool success = robot_control_msgs__srv__ControlCommand_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control_msgs__srv__ControlCommand_Request__destroy(robot_control_msgs__srv__ControlCommand_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control_msgs__srv__ControlCommand_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control_msgs__srv__ControlCommand_Request__Sequence__init(robot_control_msgs__srv__ControlCommand_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Request * data = NULL;

  if (size) {
    data = (robot_control_msgs__srv__ControlCommand_Request *)allocator.zero_allocate(size, sizeof(robot_control_msgs__srv__ControlCommand_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control_msgs__srv__ControlCommand_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control_msgs__srv__ControlCommand_Request__fini(&data[i - 1]);
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
robot_control_msgs__srv__ControlCommand_Request__Sequence__fini(robot_control_msgs__srv__ControlCommand_Request__Sequence * array)
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
      robot_control_msgs__srv__ControlCommand_Request__fini(&array->data[i]);
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

robot_control_msgs__srv__ControlCommand_Request__Sequence *
robot_control_msgs__srv__ControlCommand_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Request__Sequence * array = (robot_control_msgs__srv__ControlCommand_Request__Sequence *)allocator.allocate(sizeof(robot_control_msgs__srv__ControlCommand_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control_msgs__srv__ControlCommand_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control_msgs__srv__ControlCommand_Request__Sequence__destroy(robot_control_msgs__srv__ControlCommand_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control_msgs__srv__ControlCommand_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control_msgs__srv__ControlCommand_Request__Sequence__are_equal(const robot_control_msgs__srv__ControlCommand_Request__Sequence * lhs, const robot_control_msgs__srv__ControlCommand_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control_msgs__srv__ControlCommand_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control_msgs__srv__ControlCommand_Request__Sequence__copy(
  const robot_control_msgs__srv__ControlCommand_Request__Sequence * input,
  robot_control_msgs__srv__ControlCommand_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_control_msgs__srv__ControlCommand_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control_msgs__srv__ControlCommand_Request * data =
      (robot_control_msgs__srv__ControlCommand_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control_msgs__srv__ControlCommand_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control_msgs__srv__ControlCommand_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control_msgs__srv__ControlCommand_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
robot_control_msgs__srv__ControlCommand_Response__init(robot_control_msgs__srv__ControlCommand_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
robot_control_msgs__srv__ControlCommand_Response__fini(robot_control_msgs__srv__ControlCommand_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
robot_control_msgs__srv__ControlCommand_Response__are_equal(const robot_control_msgs__srv__ControlCommand_Response * lhs, const robot_control_msgs__srv__ControlCommand_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
robot_control_msgs__srv__ControlCommand_Response__copy(
  const robot_control_msgs__srv__ControlCommand_Response * input,
  robot_control_msgs__srv__ControlCommand_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

robot_control_msgs__srv__ControlCommand_Response *
robot_control_msgs__srv__ControlCommand_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Response * msg = (robot_control_msgs__srv__ControlCommand_Response *)allocator.allocate(sizeof(robot_control_msgs__srv__ControlCommand_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control_msgs__srv__ControlCommand_Response));
  bool success = robot_control_msgs__srv__ControlCommand_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control_msgs__srv__ControlCommand_Response__destroy(robot_control_msgs__srv__ControlCommand_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control_msgs__srv__ControlCommand_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control_msgs__srv__ControlCommand_Response__Sequence__init(robot_control_msgs__srv__ControlCommand_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Response * data = NULL;

  if (size) {
    data = (robot_control_msgs__srv__ControlCommand_Response *)allocator.zero_allocate(size, sizeof(robot_control_msgs__srv__ControlCommand_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control_msgs__srv__ControlCommand_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control_msgs__srv__ControlCommand_Response__fini(&data[i - 1]);
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
robot_control_msgs__srv__ControlCommand_Response__Sequence__fini(robot_control_msgs__srv__ControlCommand_Response__Sequence * array)
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
      robot_control_msgs__srv__ControlCommand_Response__fini(&array->data[i]);
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

robot_control_msgs__srv__ControlCommand_Response__Sequence *
robot_control_msgs__srv__ControlCommand_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Response__Sequence * array = (robot_control_msgs__srv__ControlCommand_Response__Sequence *)allocator.allocate(sizeof(robot_control_msgs__srv__ControlCommand_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control_msgs__srv__ControlCommand_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control_msgs__srv__ControlCommand_Response__Sequence__destroy(robot_control_msgs__srv__ControlCommand_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control_msgs__srv__ControlCommand_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control_msgs__srv__ControlCommand_Response__Sequence__are_equal(const robot_control_msgs__srv__ControlCommand_Response__Sequence * lhs, const robot_control_msgs__srv__ControlCommand_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control_msgs__srv__ControlCommand_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control_msgs__srv__ControlCommand_Response__Sequence__copy(
  const robot_control_msgs__srv__ControlCommand_Response__Sequence * input,
  robot_control_msgs__srv__ControlCommand_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_control_msgs__srv__ControlCommand_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control_msgs__srv__ControlCommand_Response * data =
      (robot_control_msgs__srv__ControlCommand_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control_msgs__srv__ControlCommand_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control_msgs__srv__ControlCommand_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control_msgs__srv__ControlCommand_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robot_control_msgs/srv/detail/control_command__functions.h"

bool
robot_control_msgs__srv__ControlCommand_Event__init(robot_control_msgs__srv__ControlCommand_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robot_control_msgs__srv__ControlCommand_Event__fini(msg);
    return false;
  }
  // request
  if (!robot_control_msgs__srv__ControlCommand_Request__Sequence__init(&msg->request, 0)) {
    robot_control_msgs__srv__ControlCommand_Event__fini(msg);
    return false;
  }
  // response
  if (!robot_control_msgs__srv__ControlCommand_Response__Sequence__init(&msg->response, 0)) {
    robot_control_msgs__srv__ControlCommand_Event__fini(msg);
    return false;
  }
  return true;
}

void
robot_control_msgs__srv__ControlCommand_Event__fini(robot_control_msgs__srv__ControlCommand_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robot_control_msgs__srv__ControlCommand_Request__Sequence__fini(&msg->request);
  // response
  robot_control_msgs__srv__ControlCommand_Response__Sequence__fini(&msg->response);
}

bool
robot_control_msgs__srv__ControlCommand_Event__are_equal(const robot_control_msgs__srv__ControlCommand_Event * lhs, const robot_control_msgs__srv__ControlCommand_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robot_control_msgs__srv__ControlCommand_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robot_control_msgs__srv__ControlCommand_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robot_control_msgs__srv__ControlCommand_Event__copy(
  const robot_control_msgs__srv__ControlCommand_Event * input,
  robot_control_msgs__srv__ControlCommand_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robot_control_msgs__srv__ControlCommand_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robot_control_msgs__srv__ControlCommand_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robot_control_msgs__srv__ControlCommand_Event *
robot_control_msgs__srv__ControlCommand_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Event * msg = (robot_control_msgs__srv__ControlCommand_Event *)allocator.allocate(sizeof(robot_control_msgs__srv__ControlCommand_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control_msgs__srv__ControlCommand_Event));
  bool success = robot_control_msgs__srv__ControlCommand_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control_msgs__srv__ControlCommand_Event__destroy(robot_control_msgs__srv__ControlCommand_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control_msgs__srv__ControlCommand_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control_msgs__srv__ControlCommand_Event__Sequence__init(robot_control_msgs__srv__ControlCommand_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Event * data = NULL;

  if (size) {
    data = (robot_control_msgs__srv__ControlCommand_Event *)allocator.zero_allocate(size, sizeof(robot_control_msgs__srv__ControlCommand_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control_msgs__srv__ControlCommand_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control_msgs__srv__ControlCommand_Event__fini(&data[i - 1]);
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
robot_control_msgs__srv__ControlCommand_Event__Sequence__fini(robot_control_msgs__srv__ControlCommand_Event__Sequence * array)
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
      robot_control_msgs__srv__ControlCommand_Event__fini(&array->data[i]);
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

robot_control_msgs__srv__ControlCommand_Event__Sequence *
robot_control_msgs__srv__ControlCommand_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control_msgs__srv__ControlCommand_Event__Sequence * array = (robot_control_msgs__srv__ControlCommand_Event__Sequence *)allocator.allocate(sizeof(robot_control_msgs__srv__ControlCommand_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control_msgs__srv__ControlCommand_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control_msgs__srv__ControlCommand_Event__Sequence__destroy(robot_control_msgs__srv__ControlCommand_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control_msgs__srv__ControlCommand_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control_msgs__srv__ControlCommand_Event__Sequence__are_equal(const robot_control_msgs__srv__ControlCommand_Event__Sequence * lhs, const robot_control_msgs__srv__ControlCommand_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control_msgs__srv__ControlCommand_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control_msgs__srv__ControlCommand_Event__Sequence__copy(
  const robot_control_msgs__srv__ControlCommand_Event__Sequence * input,
  robot_control_msgs__srv__ControlCommand_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_control_msgs__srv__ControlCommand_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control_msgs__srv__ControlCommand_Event * data =
      (robot_control_msgs__srv__ControlCommand_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control_msgs__srv__ControlCommand_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control_msgs__srv__ControlCommand_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control_msgs__srv__ControlCommand_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
