#pragma once
#ifndef _DM_IMU_DRIVER_HPP_
#define _DM_IMU_DRIVER_HPP_

// 不好的包含
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>


// C++ Libraries
#include <atomic>
#include <Eigen/Geometry>

#include "serial/serial.h"  // 包含串口头文件


namespace dm_imu_driver
{
  struct __attribute__((packed)) IMU_Receive_Frame
  {
    uint8_t FrameHeader1;
    uint8_t flag1;
    uint8_t slave_id1;
    uint8_t reg_acc;
    uint32_t accx_u32;
    uint32_t accy_u32;
    uint32_t accz_u32;
    uint16_t crc1;
    uint8_t FrameEnd1;

    uint8_t FrameHeader2;
    uint8_t flag2;
    uint8_t slave_id2;
    uint8_t reg_gyro;
    uint32_t gyrox_u32;
    uint32_t gyroy_u32;
    uint32_t gyroz_u32;
    uint16_t crc2;
    uint8_t FrameEnd2;

    uint8_t FrameHeader3;
    uint8_t flag3;
    uint8_t slave_id3;
    uint8_t reg_euler; // r-p-y
    uint32_t roll_u32;
    uint32_t pitch_u32;
    uint32_t yaw_u32;
    uint16_t crc3;
    uint8_t FrameEnd3;
  };

  typedef struct
  {
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
    float roll;
    float pitch;
    float yaw;
  } IMU_Data;

  class DM_imu_driver : public rclcpp::Node
  {
    public:
      DM_imu_driver(const rclcpp::NodeOptions &option);
      ~DM_imu_driver();

      Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

    private:
      struct IMUData
      {
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
      };

      // funcs
      void init_serial();
      void get_imu_data_thread();
      void declare_params();
      void read_params();

      // params
      std::string header_frame_id;
      std::string child_frame_id;
      std::string device_name;
      int baud_rate;
      double timestamp_offset_ = 0.0;

      // thread
      serial::Serial serial_;
      std::thread rec_thread_;
      std::atomic<bool> stop_thread_{false};

      IMU_Receive_Frame receive_data{}; // receive data frame
      IMU_Data data{};

      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  };

} // namespace dm_imu_driver
#endif
