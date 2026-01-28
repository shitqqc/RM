#include "camera_sync/camera_sync_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace camera_sync
{

CameraSyncNode::CameraSyncNode(const rclcpp::NodeOptions & options)
: Node("camera_sync_node", options),
  synced_count_(0),
  hik_received_(0),
  left_received_(0),
  right_received_(0)
{
  // 声明参数
  declare_parameter("max_time_diff_ms", 300.0);  // 默认 300ms
  declare_parameter("buffer_size", 50);           // 默认 50
  declare_parameter("use_sensor_data_qos", true);

  // 获取参数
  max_time_diff_ms_ = get_parameter("max_time_diff_ms").as_double();
  buffer_size_ = get_parameter("buffer_size").as_int();
  use_sensor_data_qos_ = get_parameter("use_sensor_data_qos").as_bool();

  RCLCPP_INFO(get_logger(), "Starting Camera Sync Node (HIK Master Sync Mode)");
  RCLCPP_INFO(get_logger(), "Max time difference: %.2f ms", max_time_diff_ms_);
  RCLCPP_INFO(get_logger(), "Buffer size: %d", buffer_size_);

  init();
}

void CameraSyncNode::init()
{
  // 设置 QoS
  auto qos = use_sensor_data_qos_ ? 
    rclcpp::SensorDataQoS() : 
    rclcpp::QoS(rclcpp::KeepLast(10));

  // 创建图像订阅器
  hik_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", qos,
    std::bind(&CameraSyncNode::hikCallback, this, std::placeholders::_1));
  
  usb_left_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/left/image_raw", qos,
    std::bind(&CameraSyncNode::usbLeftCallback, this, std::placeholders::_1));
  
  usb_right_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/right/image_raw", qos,
    std::bind(&CameraSyncNode::usbRightCallback, this, std::placeholders::_1));

  // 创建 camera_info 订阅器
  hik_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", qos,
    std::bind(&CameraSyncNode::hikInfoCallback, this, std::placeholders::_1));
  
  usb_left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/left/camera_info", qos,
    std::bind(&CameraSyncNode::usbLeftInfoCallback, this, std::placeholders::_1));
  
  usb_right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/right/camera_info", qos,
    std::bind(&CameraSyncNode::usbRightInfoCallback, this, std::placeholders::_1));

  // 创建图像发布器
  hik_pub_ = create_publisher<sensor_msgs::msg::Image>("sync/hik/image", 10);
  usb_left_pub_ = create_publisher<sensor_msgs::msg::Image>("sync/usb_left/image", 10);
  usb_right_pub_ = create_publisher<sensor_msgs::msg::Image>("sync/usb_right/image", 10);

  // 创建 camera_info 发布器
  hik_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("sync/hik/camera_info", 10);
  usb_left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("sync/usb_left/camera_info", 10);
  usb_right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("sync/usb_right/camera_info", 10);

  // 创建定时器，每 5ms 尝试同步一次（作为备用，主要靠HIK回调触发）
  sync_timer_ = create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&CameraSyncNode::trySyncCallback, this));

  last_sync_time_ = now();
  last_stats_time_ = now();
  
  RCLCPP_INFO(get_logger(), "Camera sync node initialized (HIK Master Sync, triggered on HIK frame arrival)");
}

void CameraSyncNode::hikCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    hik_buffer_.push_back(msg);
    hik_received_++;
    
    // 限制缓冲区大小
    while (hik_buffer_.size() > static_cast<size_t>(buffer_size_)) {
      hik_buffer_.pop_front();
    }
  }
  
  // HIK相机收到新帧时立即触发同步尝试（主相机优先策略）
  // 这样可以确保主相机的帧率不被降低
  trySyncCallback();
}

void CameraSyncNode::usbLeftCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  usb_left_buffer_.push_back(msg);
  left_received_++;
  
  while (usb_left_buffer_.size() > static_cast<size_t>(buffer_size_)) {
    usb_left_buffer_.pop_front();
  }
}

void CameraSyncNode::usbRightCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  usb_right_buffer_.push_back(msg);
  right_received_++;
  
  while (usb_right_buffer_.size() > static_cast<size_t>(buffer_size_)) {
    usb_right_buffer_.pop_front();
  }
}

void CameraSyncNode::hikInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  hik_info_buffer_.push_back(msg);
  
  while (hik_info_buffer_.size() > static_cast<size_t>(buffer_size_)) {
    hik_info_buffer_.pop_front();
  }
}

void CameraSyncNode::usbLeftInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  usb_left_info_buffer_.push_back(msg);
  
  while (usb_left_info_buffer_.size() > static_cast<size_t>(buffer_size_)) {
    usb_left_info_buffer_.pop_front();
  }
}

void CameraSyncNode::usbRightInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  usb_right_info_buffer_.push_back(msg);
  
  while (usb_right_info_buffer_.size() > static_cast<size_t>(buffer_size_)) {
    usb_right_info_buffer_.pop_front();
  }
}

void CameraSyncNode::trySyncCallback()
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  
  // 主相机（HIK）必须有数据
  if (hik_buffer_.empty()) {
    return;
  }

  // 取最旧的 HIK 图像作为参考（FIFO 策略）
  auto hik_msg = hik_buffer_.front();
  auto hik_time = rclcpp::Time(hik_msg->header.stamp);

  // 在 USB 缓冲区中找最接近的图像
  sensor_msgs::msg::Image::SharedPtr best_left = nullptr;
  sensor_msgs::msg::Image::SharedPtr best_right = nullptr;
  double best_left_diff = std::numeric_limits<double>::max();
  double best_right_diff = std::numeric_limits<double>::max();
  size_t best_left_idx = 0;
  size_t best_right_idx = 0;

  // 查找最接近的左相机图像（如果缓冲区有数据）
  if (!usb_left_buffer_.empty()) {
    for (size_t i = 0; i < usb_left_buffer_.size(); ++i) {
      auto left_time = rclcpp::Time(usb_left_buffer_[i]->header.stamp);
      double diff = std::abs((hik_time - left_time).seconds() * 1000.0);
      if (diff < best_left_diff) {
        best_left_diff = diff;
        best_left = usb_left_buffer_[i];
        best_left_idx = i;
      }
    }
  }

  // 查找最接近的右相机图像（如果缓冲区有数据）
  if (!usb_right_buffer_.empty()) {
    for (size_t i = 0; i < usb_right_buffer_.size(); ++i) {
      auto right_time = rclcpp::Time(usb_right_buffer_[i]->header.stamp);
      double diff = std::abs((hik_time - right_time).seconds() * 1000.0);
      if (diff < best_right_diff) {
        best_right_diff = diff;
        best_right = usb_right_buffer_[i];
        best_right_idx = i;
      }
    }
  }

  // 判断是否找到匹配的USB相机帧
  bool left_matched = best_left && best_left_diff < max_time_diff_ms_;
  bool right_matched = best_right && best_right_diff < max_time_diff_ms_;
  bool all_matched = left_matched && right_matched;

  // 总是发布主相机（HIK）图像，确保帧率不降低
  hik_pub_->publish(*hik_msg);
  
  // 发布对应的 camera_info（如果有的话）
  if (!hik_info_buffer_.empty()) {
    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*hik_info_buffer_.front());
    info->header.stamp = hik_msg->header.stamp;  // 使用图像的时间戳
    hik_info_pub_->publish(*info);
  }

  // 如果找到匹配的USB相机帧，也发布它们
  if (left_matched) {
    usb_left_pub_->publish(*best_left);
    if (!usb_left_info_buffer_.empty()) {
      auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*usb_left_info_buffer_.front());
      info->header.stamp = best_left->header.stamp;
      usb_left_info_pub_->publish(*info);
    }
  }
  
  if (right_matched) {
    usb_right_pub_->publish(*best_right);
    if (!usb_right_info_buffer_.empty()) {
      auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*usb_right_info_buffer_.front());
      info->header.stamp = best_right->header.stamp;
      usb_right_info_pub_->publish(*info);
    }
  }

  // 只有所有相机都匹配时才计数为同步成功
  if (all_matched) {
    synced_count_++;
  }

  // 移除已使用的HIK消息
  hik_buffer_.pop_front();
  
  // 移除已匹配的左相机缓冲区中所有早于或等于已使用消息的图像
  if (left_matched && best_left_idx < usb_left_buffer_.size()) {
    usb_left_buffer_.erase(usb_left_buffer_.begin(), usb_left_buffer_.begin() + best_left_idx + 1);
  }
  
  // 移除已匹配的右相机缓冲区中所有早于或等于已使用消息的图像
  if (right_matched && best_right_idx < usb_right_buffer_.size()) {
    usb_right_buffer_.erase(usb_right_buffer_.begin(), usb_right_buffer_.begin() + best_right_idx + 1);
  }

  // 每 30 帧输出一次统计信息
  if (synced_count_ % 30 == 0 && synced_count_ > 0) {
    auto current_time = now();
    double duration = (current_time - last_sync_time_).seconds();
    double fps = 30.0 / duration;
    
    RCLCPP_INFO(
      get_logger(),
      "Synced %ld frames (%.1f fps) | Time diff - Left: %.2f ms, Right: %.2f ms | CameraInfo: HIK=%zu, L=%zu, R=%zu",
      synced_count_, fps, 
      left_matched ? best_left_diff : -1.0, 
      right_matched ? best_right_diff : -1.0,
      hik_info_buffer_.size(), usb_left_info_buffer_.size(), usb_right_info_buffer_.size()
    );
    
    last_sync_time_ = current_time;
  }

  // 每 5 秒输出一次接收统计
  auto current_time = now();
  if ((current_time - last_stats_time_).seconds() > 5.0) {
    double success_rate = (synced_count_ * 100.0) / std::max(hik_received_, 1L);
    RCLCPP_INFO(
      get_logger(),
      "Stats - Received: HIK=%ld, Left=%ld, Right=%ld | Synced=%ld (%.1f%%) | Buffer: HIK=%zu, Left=%zu, Right=%zu",
      hik_received_, left_received_, right_received_, synced_count_, success_rate,
      hik_buffer_.size(), usb_left_buffer_.size(), usb_right_buffer_.size()
    );
    last_stats_time_ = current_time;
  }
}

}  // namespace camera_sync

RCLCPP_COMPONENTS_REGISTER_NODE(camera_sync::CameraSyncNode)
