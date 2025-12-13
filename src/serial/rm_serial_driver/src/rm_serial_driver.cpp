// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // // TF broadcaster
  // timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // // Create Publisher
  // latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    gimbal_pub =
        this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);

    try {
      serial_driver_->init_port(device_name_, *device_config_);
      if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
        receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  navi_status_=1;
  sentry_cmd=0;
  navi_status_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "navi/status", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::statusCallback, this, std::placeholders::_1));
  navi_sentry_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "navi/sentry_cmd", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sentryCallback, this, std::placeholders::_1));
  // Create Subscription
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendNavData, this, std::placeholders::_1));

  gimbal_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "gimbal_topic", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendVisionData, this, std::placeholders::_1));

    //tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  if(rclcpp::ok())
  {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x6A) {
        data.reserve(sizeof(NaviReceivePacket));
      }
      else if(header[0] == 0X5A)
      {
        data.reserve(sizeof(VisionReceivePacket));
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
  } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      throw ex;
    }
  }
  // data.reserve(sizeof(ReceivePacket));
  // RCLCPP_INFO(this->get_logger(), "In receive data.");

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);
      // RCLCPP_INFO(this->get_logger(), "Receive header.");
      if (header[0] == 0x6A) {
        data.resize(sizeof(NaviReceivePacket) - 1);
        // data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        NaviReceivePacket packet = fromVector < NaviReceivePacket>(data);
        // ReceivePacket packet = fromVector < ReceivePacket>(data);

        // RCLCPP_INFO(this->get_logger(), "In receive navi data.");

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // geometry_msgs::msg::TransformStamped t;
          // timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          // t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          // t.header.frame_id = "odom";
          // t.child_frame_id = "gimbal_link";
          // tf2::Quaternion q;
          // q.setRPY(packet.roll, packet.pitch, packet.yaw);
          // t.transform.rotation = tf2::toMsg(q);
          // tf_broadcaster_->sendTransform(t);
            sensor_msgs::msg::JointState msg;

            msg.position.resize(2);
            msg.name.resize(2);
            msg.header.stamp = now();

            msg.name[0] = "gimbal_pitch_joint";
            msg.position[0] = packet.pitch;

            msg.name[1] = "gimbal_yaw_joint";
            msg.position[1] = packet.yaw;

            // RCLCPP_INFO(
            //   this->get_logger(), "Navi gimbal_pitch: %f, gimbal_yaw: %f", packet.pitch,
            //   packet.yaw);

            // RCLCPP_INFO(this->get_logger(), "localColor : %d", packet.vision.localColor);

            gimbal_pub->publish(msg);

        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      }
      else if(header[0] == 0X5A)
      {
        data.resize(sizeof(VisionReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        VisionReceivePacket packet = fromVector<VisionReceivePacket>(data);

        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          // geometry_msgs::msg::TransformStamped t;
          // timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          // t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          // t.header.frame_id = "odom";
          // t.child_frame_id = "gimbal_link";
          // tf2::Quaternion q;
          // q.setRPY(packet.roll, packet.pitch, packet.yaw);
          // t.transform.rotation = tf2::toMsg(q);
          // tf_broadcaster_->sendTransform(t);
          // RCLCPP_INFO(this->get_logger(), "Vision localColor : %d", packet.localColor);
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } 
      else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::statusCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  navi_status_=msg->data;
}

void RMSerialDriver::sentryCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  sentry_cmd=msg->data;
}

void RMSerialDriver::sendNavData(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    if(navi_status_ == 4){
      RCLCPP_ERROR(get_logger(), "status: %d  . sentry_cmd :%d",navi_status_,sentry_cmd);
    }
    // RCLCPP_ERROR(get_logger(), "status: %d  . sentry_cmd :%d",navi_status_,sentry_cmd);

    Navi_send_packet packet;
    // packet.navi_for_robot_status = 1;
    // packet.navi_for_robot_status = static_cast<uint8_t>(navi_status_);
    // packet.navi_for_robot_status = 1;
    // packet.sentry_devision_making = static_cast<uint32_t>(sentry_cmd);
    // packet.navi.decision = static_cast<SentryDecision_e>(3);
    // packet.decision = static_cast<SentryDecision_e>(sentry_cmd + 1);
    // packet.navi.Vx = msg->linear.x;
    // packet.navi.Vy = msg->linear.y;
    // packet.navi.Wz = msg->angular.z;
    packet.decision = static_cast<SentryDecision_e>(sentry_cmd + 1);
    packet.Vx = msg->linear.x;
    packet.Vy = msg->linear.y;
    packet.Wz = msg->angular.z;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    RCLCPP_INFO(get_logger(), "send nav data: Vx:%f, Vy:%f, Wz:%f", packet.Vx, packet.Vy, packet.Wz);

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);
    // RCLCPP_INFO(get_logger(), "send data: %f", packet.Vx);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendVisionData(const std_msgs::msg::Float32::SharedPtr msg)
{
  try{
    Vision_send_packet packet;
    packet.vx = msg->data;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);
    RCLCPP_INFO(get_logger(), "send vx data: %f", packet.vx);

    serial_driver_->port()->send(data);
  }catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("nav_device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)

