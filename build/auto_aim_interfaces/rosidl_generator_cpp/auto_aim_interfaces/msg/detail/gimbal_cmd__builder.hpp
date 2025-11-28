// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/gimbal_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_GimbalCmd_pitch_acc
{
public:
  explicit Init_GimbalCmd_pitch_acc(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::GimbalCmd pitch_acc(::auto_aim_interfaces::msg::GimbalCmd::_pitch_acc_type arg)
  {
    msg_.pitch_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_pitch_vel
{
public:
  explicit Init_GimbalCmd_pitch_vel(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_pitch_acc pitch_vel(::auto_aim_interfaces::msg::GimbalCmd::_pitch_vel_type arg)
  {
    msg_.pitch_vel = std::move(arg);
    return Init_GimbalCmd_pitch_acc(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_pitch
{
public:
  explicit Init_GimbalCmd_pitch(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_pitch_vel pitch(::auto_aim_interfaces::msg::GimbalCmd::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GimbalCmd_pitch_vel(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_yaw_acc
{
public:
  explicit Init_GimbalCmd_yaw_acc(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_pitch yaw_acc(::auto_aim_interfaces::msg::GimbalCmd::_yaw_acc_type arg)
  {
    msg_.yaw_acc = std::move(arg);
    return Init_GimbalCmd_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_yaw_vel
{
public:
  explicit Init_GimbalCmd_yaw_vel(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_yaw_acc yaw_vel(::auto_aim_interfaces::msg::GimbalCmd::_yaw_vel_type arg)
  {
    msg_.yaw_vel = std::move(arg);
    return Init_GimbalCmd_yaw_acc(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_yaw
{
public:
  explicit Init_GimbalCmd_yaw(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_yaw_vel yaw(::auto_aim_interfaces::msg::GimbalCmd::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GimbalCmd_yaw_vel(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_target_pitch
{
public:
  explicit Init_GimbalCmd_target_pitch(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_yaw target_pitch(::auto_aim_interfaces::msg::GimbalCmd::_target_pitch_type arg)
  {
    msg_.target_pitch = std::move(arg);
    return Init_GimbalCmd_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_target_yaw
{
public:
  explicit Init_GimbalCmd_target_yaw(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_target_pitch target_yaw(::auto_aim_interfaces::msg::GimbalCmd::_target_yaw_type arg)
  {
    msg_.target_yaw = std::move(arg);
    return Init_GimbalCmd_target_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_fire
{
public:
  explicit Init_GimbalCmd_fire(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_target_yaw fire(::auto_aim_interfaces::msg::GimbalCmd::_fire_type arg)
  {
    msg_.fire = std::move(arg);
    return Init_GimbalCmd_target_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_control
{
public:
  explicit Init_GimbalCmd_control(::auto_aim_interfaces::msg::GimbalCmd & msg)
  : msg_(msg)
  {}
  Init_GimbalCmd_fire control(::auto_aim_interfaces::msg::GimbalCmd::_control_type arg)
  {
    msg_.control = std::move(arg);
    return Init_GimbalCmd_fire(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

class Init_GimbalCmd_header
{
public:
  Init_GimbalCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalCmd_control header(::auto_aim_interfaces::msg::GimbalCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalCmd_control(msg_);
  }

private:
  ::auto_aim_interfaces::msg::GimbalCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::GimbalCmd>()
{
  return auto_aim_interfaces::msg::builder::Init_GimbalCmd_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__BUILDER_HPP_
