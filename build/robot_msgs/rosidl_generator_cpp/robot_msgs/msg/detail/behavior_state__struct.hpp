// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/BehaviorState.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__BehaviorState __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__BehaviorState __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BehaviorState_
{
  using Type = BehaviorState_<ContainerAllocator>;

  explicit BehaviorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_state = "";
      this->task_state = "";
      this->wall_state = "";
      this->active_substate = "";
      this->task_id = "";
      this->last_error_code = 0;
      this->summary_json = "";
    }
  }

  explicit BehaviorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    system_state(_alloc),
    task_state(_alloc),
    wall_state(_alloc),
    active_substate(_alloc),
    task_id(_alloc),
    summary_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_state = "";
      this->task_state = "";
      this->wall_state = "";
      this->active_substate = "";
      this->task_id = "";
      this->last_error_code = 0;
      this->summary_json = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _system_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _system_state_type system_state;
  using _task_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_state_type task_state;
  using _wall_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _wall_state_type wall_state;
  using _active_substate_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _active_substate_type active_substate;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _last_error_code_type =
    uint16_t;
  _last_error_code_type last_error_code;
  using _summary_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _summary_json_type summary_json;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__system_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->system_state = _arg;
    return *this;
  }
  Type & set__task_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_state = _arg;
    return *this;
  }
  Type & set__wall_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->wall_state = _arg;
    return *this;
  }
  Type & set__active_substate(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->active_substate = _arg;
    return *this;
  }
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__last_error_code(
    const uint16_t & _arg)
  {
    this->last_error_code = _arg;
    return *this;
  }
  Type & set__summary_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->summary_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::BehaviorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::BehaviorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::BehaviorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::BehaviorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__BehaviorState
    std::shared_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__BehaviorState
    std::shared_ptr<robot_msgs::msg::BehaviorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BehaviorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->system_state != other.system_state) {
      return false;
    }
    if (this->task_state != other.task_state) {
      return false;
    }
    if (this->wall_state != other.wall_state) {
      return false;
    }
    if (this->active_substate != other.active_substate) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->last_error_code != other.last_error_code) {
      return false;
    }
    if (this->summary_json != other.summary_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const BehaviorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BehaviorState_

// alias to use template instance with default allocator
using BehaviorState =
  robot_msgs::msg::BehaviorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__BEHAVIOR_STATE__STRUCT_HPP_
