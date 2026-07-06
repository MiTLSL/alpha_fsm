// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/NavDebug.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__STRUCT_H_

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
// Member 'source'
// Member 'goal_type'
// Member 'status'
// Member 'details_json'
#include "rosidl_runtime_c/string.h"
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/NavDebug in the package robot_msgs.
typedef struct robot_msgs__msg__NavDebug
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String source;
  rosidl_runtime_c__String goal_type;
  geometry_msgs__msg__PoseStamped target_pose;
  float distance_remaining;
  float yaw_error;
  rosidl_runtime_c__String status;
  rosidl_runtime_c__String details_json;
} robot_msgs__msg__NavDebug;

// Struct for a sequence of robot_msgs__msg__NavDebug.
typedef struct robot_msgs__msg__NavDebug__Sequence
{
  robot_msgs__msg__NavDebug * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__NavDebug__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__STRUCT_H_
