// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_H_

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

/// Struct defined in msg/GimbalCmd in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__GimbalCmd
{
  std_msgs__msg__Header header;
  bool control;
  bool fire;
  double target_yaw;
  double target_pitch;
  double yaw;
  double yaw_vel;
  double yaw_acc;
  double pitch;
  double pitch_vel;
  double pitch_acc;
} auto_aim_interfaces__msg__GimbalCmd;

// Struct for a sequence of auto_aim_interfaces__msg__GimbalCmd.
typedef struct auto_aim_interfaces__msg__GimbalCmd__Sequence
{
  auto_aim_interfaces__msg__GimbalCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__GimbalCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__GIMBAL_CMD__STRUCT_H_
