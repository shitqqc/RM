// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/gimbal_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: control
  {
    out << "control: ";
    rosidl_generator_traits::value_to_yaml(msg.control, out);
    out << ", ";
  }

  // member: fire
  {
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
    out << ", ";
  }

  // member: target_yaw
  {
    out << "target_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.target_yaw, out);
    out << ", ";
  }

  // member: target_pitch
  {
    out << "target_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.target_pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: yaw_vel
  {
    out << "yaw_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_vel, out);
    out << ", ";
  }

  // member: yaw_acc
  {
    out << "yaw_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_acc, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: pitch_vel
  {
    out << "pitch_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_vel, out);
    out << ", ";
  }

  // member: pitch_acc
  {
    out << "pitch_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalCmd & msg,
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

  // member: control
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control: ";
    rosidl_generator_traits::value_to_yaml(msg.control, out);
    out << "\n";
  }

  // member: fire
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
    out << "\n";
  }

  // member: target_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.target_yaw, out);
    out << "\n";
  }

  // member: target_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.target_pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: yaw_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_vel, out);
    out << "\n";
  }

  // member: yaw_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_acc, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: pitch_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_vel, out);
    out << "\n";
  }

  // member: pitch_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalCmd & msg, bool use_flow_style = false)
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

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::GimbalCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::GimbalCmd & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::GimbalCmd>()
{
  return "auto_aim_interfaces::msg::GimbalCmd";
}

template<>
inline const char * name<auto_aim_interfaces::msg::GimbalCmd>()
{
  return "auto_aim_interfaces/msg/GimbalCmd";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::GimbalCmd>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::GimbalCmd>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<auto_aim_interfaces::msg::GimbalCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__TRAITS_HPP_
