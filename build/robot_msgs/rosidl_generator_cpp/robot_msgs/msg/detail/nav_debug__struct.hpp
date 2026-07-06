// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/NavDebug.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__NavDebug __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__NavDebug __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavDebug_
{
  using Type = NavDebug_<ContainerAllocator>;

  explicit NavDebug_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->source = "";
      this->goal_type = "";
      this->distance_remaining = 0.0f;
      this->yaw_error = 0.0f;
      this->status = "";
      this->details_json = "";
    }
  }

  explicit NavDebug_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    source(_alloc),
    goal_type(_alloc),
    target_pose(_alloc, _init),
    status(_alloc),
    details_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->source = "";
      this->goal_type = "";
      this->distance_remaining = 0.0f;
      this->yaw_error = 0.0f;
      this->status = "";
      this->details_json = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_type source;
  using _goal_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _goal_type_type goal_type;
  using _target_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _target_pose_type target_pose;
  using _distance_remaining_type =
    float;
  _distance_remaining_type distance_remaining;
  using _yaw_error_type =
    float;
  _yaw_error_type yaw_error;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _details_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _details_json_type details_json;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source = _arg;
    return *this;
  }
  Type & set__goal_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->goal_type = _arg;
    return *this;
  }
  Type & set__target_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }
  Type & set__distance_remaining(
    const float & _arg)
  {
    this->distance_remaining = _arg;
    return *this;
  }
  Type & set__yaw_error(
    const float & _arg)
  {
    this->yaw_error = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__details_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->details_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::NavDebug_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::NavDebug_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::NavDebug_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::NavDebug_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__NavDebug
    std::shared_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__NavDebug
    std::shared_ptr<robot_msgs::msg::NavDebug_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavDebug_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->source != other.source) {
      return false;
    }
    if (this->goal_type != other.goal_type) {
      return false;
    }
    if (this->target_pose != other.target_pose) {
      return false;
    }
    if (this->distance_remaining != other.distance_remaining) {
      return false;
    }
    if (this->yaw_error != other.yaw_error) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->details_json != other.details_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavDebug_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavDebug_

// alias to use template instance with default allocator
using NavDebug =
  robot_msgs::msg::NavDebug_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__NAV_DEBUG__STRUCT_HPP_
