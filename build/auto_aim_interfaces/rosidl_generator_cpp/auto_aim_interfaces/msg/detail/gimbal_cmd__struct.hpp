// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__GimbalCmd __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__GimbalCmd __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GimbalCmd_
{
  using Type = GimbalCmd_<ContainerAllocator>;

  explicit GimbalCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control = false;
      this->fire = false;
      this->target_yaw = 0.0;
      this->target_pitch = 0.0;
      this->yaw = 0.0;
      this->yaw_vel = 0.0;
      this->yaw_acc = 0.0;
      this->pitch = 0.0;
      this->pitch_vel = 0.0;
      this->pitch_acc = 0.0;
    }
  }

  explicit GimbalCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control = false;
      this->fire = false;
      this->target_yaw = 0.0;
      this->target_pitch = 0.0;
      this->yaw = 0.0;
      this->yaw_vel = 0.0;
      this->yaw_acc = 0.0;
      this->pitch = 0.0;
      this->pitch_vel = 0.0;
      this->pitch_acc = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _control_type =
    bool;
  _control_type control;
  using _fire_type =
    bool;
  _fire_type fire;
  using _target_yaw_type =
    double;
  _target_yaw_type target_yaw;
  using _target_pitch_type =
    double;
  _target_pitch_type target_pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _yaw_vel_type =
    double;
  _yaw_vel_type yaw_vel;
  using _yaw_acc_type =
    double;
  _yaw_acc_type yaw_acc;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _pitch_vel_type =
    double;
  _pitch_vel_type pitch_vel;
  using _pitch_acc_type =
    double;
  _pitch_acc_type pitch_acc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__control(
    const bool & _arg)
  {
    this->control = _arg;
    return *this;
  }
  Type & set__fire(
    const bool & _arg)
  {
    this->fire = _arg;
    return *this;
  }
  Type & set__target_yaw(
    const double & _arg)
  {
    this->target_yaw = _arg;
    return *this;
  }
  Type & set__target_pitch(
    const double & _arg)
  {
    this->target_pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__yaw_vel(
    const double & _arg)
  {
    this->yaw_vel = _arg;
    return *this;
  }
  Type & set__yaw_acc(
    const double & _arg)
  {
    this->yaw_acc = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__pitch_vel(
    const double & _arg)
  {
    this->pitch_vel = _arg;
    return *this;
  }
  Type & set__pitch_acc(
    const double & _arg)
  {
    this->pitch_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__GimbalCmd
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__GimbalCmd
    std::shared_ptr<auto_aim_interfaces::msg::GimbalCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GimbalCmd_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->control != other.control) {
      return false;
    }
    if (this->fire != other.fire) {
      return false;
    }
    if (this->target_yaw != other.target_yaw) {
      return false;
    }
    if (this->target_pitch != other.target_pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->yaw_vel != other.yaw_vel) {
      return false;
    }
    if (this->yaw_acc != other.yaw_acc) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->pitch_vel != other.pitch_vel) {
      return false;
    }
    if (this->pitch_acc != other.pitch_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const GimbalCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GimbalCmd_

// alias to use template instance with default allocator
using GimbalCmd =
  auto_aim_interfaces::msg::GimbalCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_HPP_
