// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/BehaviorState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'system_state'
// Member 'task_state'
// Member 'wall_state'
// Member 'active_substate'
// Member 'task_id'
// Member 'summary_json'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BehaviorState in the package robot_msgs.
typedef struct robot_msgs__msg__BehaviorState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String system_state;
  rosidl_runtime_c__String task_state;
  rosidl_runtime_c__String wall_state;
  rosidl_runtime_c__String active_substate;
  rosidl_runtime_c__String task_id;
  uint16_t last_error_code;
  rosidl_runtime_c__String summary_json;
} robot_msgs__msg__BehaviorState;

// Struct for a sequence of robot_msgs__msg__BehaviorState.
typedef struct robot_msgs__msg__BehaviorState__Sequence
{
  robot_msgs__msg__BehaviorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__BehaviorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__STRUCT_H_
