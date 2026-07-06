// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/BehaviorState.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/behavior_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `system_state`
// Member `task_state`
// Member `wall_state`
// Member `active_substate`
// Member `task_id`
// Member `summary_json`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_msgs__msg__BehaviorState__init(robot_msgs__msg__BehaviorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  // system_state
  if (!rosidl_runtime_c__String__init(&msg->system_state)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  // task_state
  if (!rosidl_runtime_c__String__init(&msg->task_state)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  // wall_state
  if (!rosidl_runtime_c__String__init(&msg->wall_state)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  // active_substate
  if (!rosidl_runtime_c__String__init(&msg->active_substate)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__init(&msg->task_id)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  // last_error_code
  // summary_json
  if (!rosidl_runtime_c__String__init(&msg->summary_json)) {
    robot_msgs__msg__BehaviorState__fini(msg);
    return false;
  }
  return true;
}

void
robot_msgs__msg__BehaviorState__fini(robot_msgs__msg__BehaviorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // system_state
  rosidl_runtime_c__String__fini(&msg->system_state);
  // task_state
  rosidl_runtime_c__String__fini(&msg->task_state);
  // wall_state
  rosidl_runtime_c__String__fini(&msg->wall_state);
  // active_substate
  rosidl_runtime_c__String__fini(&msg->active_substate);
  // task_id
  rosidl_runtime_c__String__fini(&msg->task_id);
  // last_error_code
  // summary_json
  rosidl_runtime_c__String__fini(&msg->summary_json);
}

bool
robot_msgs__msg__BehaviorState__are_equal(const robot_msgs__msg__BehaviorState * lhs, const robot_msgs__msg__BehaviorState * rhs)
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
  // system_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->system_state), &(rhs->system_state)))
  {
    return false;
  }
  // task_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_state), &(rhs->task_state)))
  {
    return false;
  }
  // wall_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->wall_state), &(rhs->wall_state)))
  {
    return false;
  }
  // active_substate
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->active_substate), &(rhs->active_substate)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_id), &(rhs->task_id)))
  {
    return false;
  }
  // last_error_code
  if (lhs->last_error_code != rhs->last_error_code) {
    return false;
  }
  // summary_json
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->summary_json), &(rhs->summary_json)))
  {
    return false;
  }
  return true;
}

bool
robot_msgs__msg__BehaviorState__copy(
  const robot_msgs__msg__BehaviorState * input,
  robot_msgs__msg__BehaviorState * output)
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
  // system_state
  if (!rosidl_runtime_c__String__copy(
      &(input->system_state), &(output->system_state)))
  {
    return false;
  }
  // task_state
  if (!rosidl_runtime_c__String__copy(
      &(input->task_state), &(output->task_state)))
  {
    return false;
  }
  // wall_state
  if (!rosidl_runtime_c__String__copy(
      &(input->wall_state), &(output->wall_state)))
  {
    return false;
  }
  // active_substate
  if (!rosidl_runtime_c__String__copy(
      &(input->active_substate), &(output->active_substate)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__copy(
      &(input->task_id), &(output->task_id)))
  {
    return false;
  }
  // last_error_code
  output->last_error_code = input->last_error_code;
  // summary_json
  if (!rosidl_runtime_c__String__copy(
      &(input->summary_json), &(output->summary_json)))
  {
    return false;
  }
  return true;
}

robot_msgs__msg__BehaviorState *
robot_msgs__msg__BehaviorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__BehaviorState * msg = (robot_msgs__msg__BehaviorState *)allocator.allocate(sizeof(robot_msgs__msg__BehaviorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__BehaviorState));
  bool success = robot_msgs__msg__BehaviorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__BehaviorState__destroy(robot_msgs__msg__BehaviorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__BehaviorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__BehaviorState__Sequence__init(robot_msgs__msg__BehaviorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__BehaviorState * data = NULL;

  if (size) {
    data = (robot_msgs__msg__BehaviorState *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__BehaviorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__BehaviorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__BehaviorState__fini(&data[i - 1]);
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
robot_msgs__msg__BehaviorState__Sequence__fini(robot_msgs__msg__BehaviorState__Sequence * array)
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
      robot_msgs__msg__BehaviorState__fini(&array->data[i]);
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

robot_msgs__msg__BehaviorState__Sequence *
robot_msgs__msg__BehaviorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__BehaviorState__Sequence * array = (robot_msgs__msg__BehaviorState__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__BehaviorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__BehaviorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__BehaviorState__Sequence__destroy(robot_msgs__msg__BehaviorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__BehaviorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__BehaviorState__Sequence__are_equal(const robot_msgs__msg__BehaviorState__Sequence * lhs, const robot_msgs__msg__BehaviorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__BehaviorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__BehaviorState__Sequence__copy(
  const robot_msgs__msg__BehaviorState__Sequence * input,
  robot_msgs__msg__BehaviorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__BehaviorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__BehaviorState * data =
      (robot_msgs__msg__BehaviorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__BehaviorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__BehaviorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__BehaviorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
