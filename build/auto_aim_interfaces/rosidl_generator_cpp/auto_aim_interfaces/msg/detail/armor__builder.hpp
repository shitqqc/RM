// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Armor.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/armor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Armor_priority
{
public:
  explicit Init_Armor_priority(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Armor priority(::auto_aim_interfaces::msg::Armor::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_dyaw
{
public:
  explicit Init_Armor_dyaw(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_priority dyaw(::auto_aim_interfaces::msg::Armor::_dyaw_type arg)
  {
    msg_.dyaw = std::move(arg);
    return Init_Armor_priority(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_yaw
{
public:
  explicit Init_Armor_yaw(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_dyaw yaw(::auto_aim_interfaces::msg::Armor::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Armor_dyaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_yaw_raw
{
public:
  explicit Init_Armor_yaw_raw(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_yaw yaw_raw(::auto_aim_interfaces::msg::Armor::_yaw_raw_type arg)
  {
    msg_.yaw_raw = std::move(arg);
    return Init_Armor_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_kpts
{
public:
  explicit Init_Armor_kpts(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_yaw_raw kpts(::auto_aim_interfaces::msg::Armor::_kpts_type arg)
  {
    msg_.kpts = std::move(arg);
    return Init_Armor_yaw_raw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_pose
{
public:
  explicit Init_Armor_pose(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_kpts pose(::auto_aim_interfaces::msg::Armor::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Armor_kpts(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_distance_to_image_center
{
public:
  explicit Init_Armor_distance_to_image_center(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_pose distance_to_image_center(::auto_aim_interfaces::msg::Armor::_distance_to_image_center_type arg)
  {
    msg_.distance_to_image_center = std::move(arg);
    return Init_Armor_pose(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_color
{
public:
  explicit Init_Armor_color(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_distance_to_image_center color(::auto_aim_interfaces::msg::Armor::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_Armor_distance_to_image_center(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_prob
{
public:
  explicit Init_Armor_prob(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_color prob(::auto_aim_interfaces::msg::Armor::_prob_type arg)
  {
    msg_.prob = std::move(arg);
    return Init_Armor_color(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_type
{
public:
  explicit Init_Armor_type(::auto_aim_interfaces::msg::Armor & msg)
  : msg_(msg)
  {}
  Init_Armor_prob type(::auto_aim_interfaces::msg::Armor::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Armor_prob(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

class Init_Armor_number
{
public:
  Init_Armor_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Armor_type number(::auto_aim_interfaces::msg::Armor::_number_type arg)
  {
    msg_.number = std::move(arg);
    return Init_Armor_type(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Armor>()
{
  return auto_aim_interfaces::msg::builder::Init_Armor_number();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ARMOR__BUILDER_HPP_
