// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/BehaviorState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/behavior_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_BehaviorState_summary_json
{
public:
  explicit Init_BehaviorState_summary_json(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::BehaviorState summary_json(::robot_msgs::msg::BehaviorState::_summary_json_type arg)
  {
    msg_.summary_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_last_error_code
{
public:
  explicit Init_BehaviorState_last_error_code(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  Init_BehaviorState_summary_json last_error_code(::robot_msgs::msg::BehaviorState::_last_error_code_type arg)
  {
    msg_.last_error_code = std::move(arg);
    return Init_BehaviorState_summary_json(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_task_id
{
public:
  explicit Init_BehaviorState_task_id(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  Init_BehaviorState_last_error_code task_id(::robot_msgs::msg::BehaviorState::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_BehaviorState_last_error_code(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_active_substate
{
public:
  explicit Init_BehaviorState_active_substate(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  Init_BehaviorState_task_id active_substate(::robot_msgs::msg::BehaviorState::_active_substate_type arg)
  {
    msg_.active_substate = std::move(arg);
    return Init_BehaviorState_task_id(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_wall_state
{
public:
  explicit Init_BehaviorState_wall_state(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  Init_BehaviorState_active_substate wall_state(::robot_msgs::msg::BehaviorState::_wall_state_type arg)
  {
    msg_.wall_state = std::move(arg);
    return Init_BehaviorState_active_substate(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_task_state
{
public:
  explicit Init_BehaviorState_task_state(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  Init_BehaviorState_wall_state task_state(::robot_msgs::msg::BehaviorState::_task_state_type arg)
  {
    msg_.task_state = std::move(arg);
    return Init_BehaviorState_wall_state(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_system_state
{
public:
  explicit Init_BehaviorState_system_state(::robot_msgs::msg::BehaviorState & msg)
  : msg_(msg)
  {}
  Init_BehaviorState_task_state system_state(::robot_msgs::msg::BehaviorState::_system_state_type arg)
  {
    msg_.system_state = std::move(arg);
    return Init_BehaviorState_task_state(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

class Init_BehaviorState_header
{
public:
  Init_BehaviorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BehaviorState_system_state header(::robot_msgs::msg::BehaviorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BehaviorState_system_state(msg_);
  }

private:
  ::robot_msgs::msg::BehaviorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::BehaviorState>()
{
  return robot_msgs::msg::builder::Init_BehaviorState_header();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__BUILDER_HPP_
