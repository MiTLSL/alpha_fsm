// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/NavDebug.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/nav_debug__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_NavDebug_details_json
{
public:
  explicit Init_NavDebug_details_json(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::NavDebug details_json(::robot_msgs::msg::NavDebug::_details_json_type arg)
  {
    msg_.details_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_status
{
public:
  explicit Init_NavDebug_status(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  Init_NavDebug_details_json status(::robot_msgs::msg::NavDebug::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NavDebug_details_json(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_yaw_error
{
public:
  explicit Init_NavDebug_yaw_error(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  Init_NavDebug_status yaw_error(::robot_msgs::msg::NavDebug::_yaw_error_type arg)
  {
    msg_.yaw_error = std::move(arg);
    return Init_NavDebug_status(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_distance_remaining
{
public:
  explicit Init_NavDebug_distance_remaining(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  Init_NavDebug_yaw_error distance_remaining(::robot_msgs::msg::NavDebug::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return Init_NavDebug_yaw_error(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_target_pose
{
public:
  explicit Init_NavDebug_target_pose(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  Init_NavDebug_distance_remaining target_pose(::robot_msgs::msg::NavDebug::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_NavDebug_distance_remaining(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_goal_type
{
public:
  explicit Init_NavDebug_goal_type(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  Init_NavDebug_target_pose goal_type(::robot_msgs::msg::NavDebug::_goal_type_type arg)
  {
    msg_.goal_type = std::move(arg);
    return Init_NavDebug_target_pose(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_source
{
public:
  explicit Init_NavDebug_source(::robot_msgs::msg::NavDebug & msg)
  : msg_(msg)
  {}
  Init_NavDebug_goal_type source(::robot_msgs::msg::NavDebug::_source_type arg)
  {
    msg_.source = std::move(arg);
    return Init_NavDebug_goal_type(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

class Init_NavDebug_header
{
public:
  Init_NavDebug_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavDebug_source header(::robot_msgs::msg::NavDebug::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavDebug_source(msg_);
  }

private:
  ::robot_msgs::msg::NavDebug msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::NavDebug>()
{
  return robot_msgs::msg::builder::Init_NavDebug_header();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__BUILDER_HPP_
