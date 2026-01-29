// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

//tf
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendNavData(geometry_msgs::msg::Twist::SharedPtr msg);
  void sendVisionData(std_msgs::msg::Float32::SharedPtr msg);
  void sendVisionPacketData(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  void reopenPort();

  void statusCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void sentryCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void yawOffsetCallback(const std_msgs::msg::Float32::SharedPtr msg);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // // Broadcast tf from odom to gimbal_link
  // double timestamp_offset_ = 0;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr navi_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr navi_sentry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr vision_send_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_offset_sub_;

  int navi_status_,sentry_cmd;
  // // For debug usage
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr vision_pub;

  std::thread receive_thread_;

  //tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  geometry_msgs::msg::TransformStamped t_;

  // 来自决策模块的 yaw 偏移（弧度），用于修正视觉发送回下位机的目标角度
  double yaw_offset_rad_ = 0.0;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_