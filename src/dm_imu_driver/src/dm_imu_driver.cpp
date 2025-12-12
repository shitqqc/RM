// C++ Librarys
#include <unistd.h>
#include <thread>
// #include <queue>

// ros2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>


#include "serial/serial.h"
#include "dm_imu_driver/dm_imu_driver.hpp"
// #include "tools/thread_safe_queue.hpp"
#include "serial/crc.hpp"

namespace dm_imu_driver
{
  DM_imu_driver::DM_imu_driver(const rclcpp::NodeOptions &options) : Node("imu_driver", options)
  {
    declare_params();
    read_params();

    init_serial();

    // this->declare_parameter("timestamp_offset", 0.0);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    rec_thread_ = std::thread(&DM_imu_driver::get_imu_data_thread, this);

    RCLCPP_INFO(this->get_logger(),"IMU_Driver initialized !");
  }

  DM_imu_driver::~DM_imu_driver()
  {
    stop_thread_ = true;
    if (rec_thread_.joinable())
    {
      rec_thread_.join();
    }
    if (serial_.isOpen())
    {
      serial_.close();
    }
  }

  void DM_imu_driver::init_serial()
  {
    try
    {
      serial_.setPort(this->device_name);
      serial_.setBaudrate(921600);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none); // default is parity_none
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
      serial_.setTimeout(time_out);
      serial_.open();
      usleep(1000000); // 1s

      RCLCPP_INFO(this->get_logger(), "[DM_imu_driver] serial port opened");
    }

    catch (serial::IOException &e)
    {
      RCLCPP_FATAL(this->get_logger(),"failed to open serial port ");
      exit(0);
    } // try -- catch
  } // init_serial

  void DM_imu_driver::get_imu_data_thread(){
    while (!stop_thread_)
    {
      if (!serial_.isOpen())
      {
        RCLCPP_WARN(this->get_logger(),"In get_imu_data_thread,imu serial port unopen");
        continue; // 也许需要这样？
      }

      serial_.read((uint8_t *)(&receive_data.FrameHeader1), 4);

      if (
          receive_data.FrameHeader1 == 0x55 && receive_data.flag1 == 0xAA &&
          receive_data.slave_id1 == 0x01 && receive_data.reg_acc == 0x01)
      {
        serial_.read((uint8_t *)(&receive_data.accx_u32), 57 - 4);

        if (tools::get_crc16((uint8_t const *)(&receive_data.FrameHeader1), 16) == receive_data.crc1)
        {
          data.accx = *((float *)(&receive_data.accx_u32));
          data.accy = *((float *)(&receive_data.accy_u32));
          data.accz = *((float *)(&receive_data.accz_u32));
        }else{
          RCLCPP_WARN(this->get_logger(),"Crc1 Error");
          continue;
        }
        if (tools::get_crc16((uint8_t const *)(&receive_data.FrameHeader2), 16) == receive_data.crc2)
        {
          data.gyrox = *((float *)(&receive_data.gyrox_u32));
          data.gyroy = *((float *)(&receive_data.gyroy_u32));
          data.gyroz = *((float *)(&receive_data.gyroz_u32));
        }else{
          RCLCPP_WARN(this->get_logger(),"Crc2 Error");
          continue;
        }
        if (tools::get_crc16((uint8_t const *)(&receive_data.FrameHeader3), 16) == receive_data.crc3)
        {
          data.roll = *((float *)(&receive_data.roll_u32));
          data.pitch = *((float *)(&receive_data.pitch_u32));
          data.yaw = *((float *)(&receive_data.yaw_u32));
        }else{
          RCLCPP_WARN(this->get_logger(),"Crc3 Error");
          continue;
        }

        // RCLCPP_INFO(this->get_logger(),"准备发送数据");
        geometry_msgs::msg::TransformStamped tf_msg;

        // 时间戳
        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
        tf_msg.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        // RCLCPP_INFO(this->get_logger(),"获取到时间戳：%d.%09d",tf_msg.header.stamp.sec,tf_msg.header.stamp.nanosec);
        
        // 坐标关系
        tf_msg.header.frame_id = header_frame_id;
        tf_msg.child_frame_id = child_frame_id;

        // 旋转
        tf2::Quaternion q;
        q.setRPY(data.roll/180*M_PI, data.pitch/180*M_PI, data.yaw/180*M_PI);
        tf_msg.transform.rotation = tf2::toMsg(q);

        // 添加平移部分
        // TODO: 添加正确的平移
        tf_msg.transform.translation.x = 1.0;
        tf_msg.transform.translation.y = 1.0;
        tf_msg.transform.translation.z = 0.0;

          tf_broadcaster_->sendTransform(tf_msg);
          // RCLCPP_INFO(this->get_logger(),"发送数据成功");
      } 
      else
      {
        RCLCPP_WARN(this->get_logger(),"failed to get correct IMU data");
      }
    }
  } // get_imu_data_thread

  void DM_imu_driver::declare_params(){
    try {
      this->declare_parameter<std::string>("header_frame_id","odom");
    } catch (rclcpp::ParameterTypeException& ex) {
      RCLCPP_ERROR(this->get_logger(),"The header-frame-id was invalid");
      throw ex;
    }
    
    try {
      this->declare_parameter<std::string>("child_frame_id","gimbal_link");
    }catch(rclcpp::ParameterTypeException &ex){
      RCLCPP_ERROR(this->get_logger(),"The child-frame-id was invalid");
      throw ex;
    }

    try {
      device_name = this->declare_parameter<std::string>("device_name", "");
    } catch (rclcpp::ParameterTypeException &ex) {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try {
      baud_rate = declare_parameter<int>("baud_rate", 0);
    } catch (rclcpp::ParameterTypeException &ex){
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }
    try{
      this->declare_parameter<double>("timestamp_offset",0.0); // 注意构造函数有一个临时的声明
    } catch (rclcpp::ParameterTypeException &ex){
      RCLCPP_ERROR(this->get_logger(),"Failed to get the timestamp_offset");
      throw ex;
    }
  }

  void DM_imu_driver::read_params(){
    header_frame_id = this->get_parameter("header_frame_id").as_string();
    child_frame_id = this->get_parameter("child_frame_id").as_string();
    device_name = this->get_parameter("device_name").as_string();
    baud_rate = this->get_parameter("baud_rate").as_int(); // get the baud rate par
  }

} // namespace dm_imu_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(dm_imu_driver::DM_imu_driver)