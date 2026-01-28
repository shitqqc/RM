#ifndef CAMERA_SYNC__CAMERA_SYNC_NODE_HPP_
#define CAMERA_SYNC__CAMERA_SYNC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>  // 添加这行
#include <deque>
#include <mutex>

namespace camera_sync
{

class CameraSyncNode : public rclcpp::Node
{
public:
  explicit CameraSyncNode(const rclcpp::NodeOptions & options);

private:
  void init();
  void hikCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void usbLeftCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void usbRightCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  // 添加 camera_info 回调
  void hikInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void usbLeftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void usbRightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  
  void trySyncCallback();

  // 图像订阅器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr hik_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr usb_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr usb_right_sub_;

  // 添加 camera_info 订阅器
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr hik_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr usb_left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr usb_right_info_sub_;

  // 图像发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hik_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr usb_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr usb_right_pub_;

  // 添加 camera_info 发布器
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr hik_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr usb_left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr usb_right_info_pub_;

  // 图像缓冲区
  std::deque<sensor_msgs::msg::Image::SharedPtr> hik_buffer_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> usb_left_buffer_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> usb_right_buffer_;

  // 添加 camera_info 缓冲区
  std::deque<sensor_msgs::msg::CameraInfo::SharedPtr> hik_info_buffer_;
  std::deque<sensor_msgs::msg::CameraInfo::SharedPtr> usb_left_info_buffer_;
  std::deque<sensor_msgs::msg::CameraInfo::SharedPtr> usb_right_info_buffer_;

  rclcpp::TimerBase::SharedPtr sync_timer_;
  std::mutex buffer_mutex_;

  // 参数
  double max_time_diff_ms_;
  int buffer_size_;
  bool use_sensor_data_qos_;

  // 统计
  long synced_count_;
  long hik_received_;
  long left_received_;
  long right_received_;
  rclcpp::Time last_sync_time_;
  rclcpp::Time last_stats_time_;
};

}  // namespace camera_sync

#endif  // CAMERA_SYNC__CAMERA_SYNC_NODE_HPP_
