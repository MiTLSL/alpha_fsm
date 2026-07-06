// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/BehaviorState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/behavior_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BehaviorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: system_state
  {
    out << "system_state: ";
    rosidl_generator_traits::value_to_yaml(msg.system_state, out);
    out << ", ";
  }

  // member: task_state
  {
    out << "task_state: ";
    rosidl_generator_traits::value_to_yaml(msg.task_state, out);
    out << ", ";
  }

  // member: wall_state
  {
    out << "wall_state: ";
    rosidl_generator_traits::value_to_yaml(msg.wall_state, out);
    out << ", ";
  }

  // member: active_substate
  {
    out << "active_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.active_substate, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: last_error_code
  {
    out << "last_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.last_error_code, out);
    out << ", ";
  }

  // member: summary_json
  {
    out << "summary_json: ";
    rosidl_generator_traits::value_to_yaml(msg.summary_json, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BehaviorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: system_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_state: ";
    rosidl_generator_traits::value_to_yaml(msg.system_state, out);
    out << "\n";
  }

  // member: task_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_state: ";
    rosidl_generator_traits::value_to_yaml(msg.task_state, out);
    out << "\n";
  }

  // member: wall_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wall_state: ";
    rosidl_generator_traits::value_to_yaml(msg.wall_state, out);
    out << "\n";
  }

  // member: active_substate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active_substate: ";
    rosidl_generator_traits::value_to_yaml(msg.active_substate, out);
    out << "\n";
  }

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: last_error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.last_error_code, out);
    out << "\n";
  }

  // member: summary_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "summary_json: ";
    rosidl_generator_traits::value_to_yaml(msg.summary_json, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BehaviorState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::msg::BehaviorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::BehaviorState & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::BehaviorState>()
{
  return "robot_msgs::msg::BehaviorState";
}

template<>
inline const char * name<robot_msgs::msg::BehaviorState>()
{
  return "robot_msgs/msg/BehaviorState";
}

template<>
struct has_fixed_size<robot_msgs::msg::BehaviorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::msg::BehaviorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::msg::BehaviorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__TRAITS_HPP_
