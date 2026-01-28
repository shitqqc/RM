#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <memory>

namespace usb_camera
{

class UsbCameraNode : public rclcpp::Node
{
public:
  explicit UsbCameraNode(const rclcpp::NodeOptions & options)
  : Node("usb_camera", options)
  {
    RCLCPP_INFO(get_logger(), "Starting USB Camera Node");

    // 声明参数
    declare_parameters();
    // 打开摄像头设备
    open_camera();
    // 设置摄像头参数
    set_camera_parameters();
    // 打印实际摄像头参数
    print_actual_camera_params();
    // 设置相机信息管理器
    setup_camera_info_manager();
    // 设置图像发布器
    setup_publisher();
    // 设置参数回调
    setup_parameter_callback();
    
    // 启动图像采集线程
    capture_thread_ = std::thread(&UsbCameraNode::capture_loop, this);
  }

  ~UsbCameraNode() override
  {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    
    std::lock_guard<std::mutex> lock(cap_mutex_);
    if (cap_.isOpened()) {
      cap_.release();
    }
    RCLCPP_INFO(get_logger(), "USB Camera Node destroyed");
  }

private:
  // 声明节点参数
  void declare_parameters()
  {
    // 摄像头设备路径（支持设备路径和ID）
    video_device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    // 图像宽度
    frame_width_ = declare_parameter<int>("frame_width", 1280);
    // 图像高度
    frame_height_ = declare_parameter<int>("frame_height", 1024);
    // 帧率
    fps_ = declare_parameter<double>("fps", 60.0);
    // 曝光值
    exposure_ = declare_parameter<double>("exposure", 100.0);
    // Gamma值
    gamma_ = declare_parameter<double>("gamma", 400.0);
    // 增益
    gain_ = declare_parameter<double>("gain", 10.0);
    // 像素格式
    pixel_format_ = declare_parameter<std::string>("pixel_format", "YUYV");
    // 相机名称（用于相机信息管理）
    camera_name_ = declare_parameter<std::string>("camera_name", "usb_camera");
    // 相机标定文件URL
    camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
    
    // 打印加载的参数值用于调试
    /*RCLCPP_INFO(get_logger(), "Loaded parameters:");
    RCLCPP_INFO(get_logger(), "  video_device: %s", video_device_.c_str());
    RCLCPP_INFO(get_logger(), "  frame_width: %d", frame_width_);
    RCLCPP_INFO(get_logger(), "  frame_height: %d", frame_height_);
    RCLCPP_INFO(get_logger(), "  fps: %.2f", fps_);
    RCLCPP_INFO(get_logger(), "  exposure: %.2f", exposure_);
    RCLCPP_INFO(get_logger(), "  gamma: %.2f", gamma_);
    RCLCPP_INFO(get_logger(), "  gain: %.2f", gain_);
    RCLCPP_INFO(get_logger(), "  pixel_format: %s", pixel_format_.c_str());
    RCLCPP_INFO(get_logger(), "  camera_name: %s", camera_name_.c_str());*/
  }

  // 打开摄像头设备
  void open_camera()
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    
    if (video_device_.find("/dev/") == 0) {
      cap_.open(video_device_, cv::CAP_V4L2);
    } else {
      try {
        int device_id = std::stoi(video_device_);
        cap_.open(device_id, cv::CAP_V4L2);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Invalid video device: %s", video_device_.c_str());
        throw;
      }
    }
    
    int retry = 0;
    while (!cap_.isOpened() && retry < 5 && rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera device %s, retrying...", video_device_.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        if (video_device_.find("/dev/") == 0) {
          cap_.open(video_device_, cv::CAP_V4L2);
        } else {
          int device_id = std::stoi(video_device_);
          cap_.open(device_id, cv::CAP_V4L2);
        }
        retry++;
    }
    
    if (!cap_.isOpened()) {
        RCLCPP_FATAL(get_logger(), "Failed to open camera device %s after %d attempts", 
                    video_device_.c_str(), retry);
        throw std::runtime_error("Camera open failed");
    }
    
    RCLCPP_INFO(get_logger(), "Successfully opened camera device %s", video_device_.c_str());
  }

  void set_camera_parameters()
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    
    if (pixel_format_ == "YUYV") {
      bool success = cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
      if (!success) {
        RCLCPP_WARN(get_logger(), "Failed to set YUYV format, trying MJPG");
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        pixel_format_ = "MJPG";
      }
    } else if (pixel_format_ == "MJPG") {
      cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    } else {
      RCLCPP_WARN(get_logger(), "Unsupported pixel format: %s, using default", pixel_format_.c_str());
    }
    
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // 手动曝光
    cap_.set(cv::CAP_PROP_EXPOSURE, exposure_);
    cap_.set(cv::CAP_PROP_GAMMA, gamma_);
    cap_.set(cv::CAP_PROP_GAIN, gain_);
    cap_.set(cv::CAP_PROP_AUTO_WB, 1);  // 关闭自动白平衡

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 打印实际摄像头参数
  void print_actual_camera_params()
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    
    int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    
    // 解码FourCC编码格式
    int fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    char pixel_format[5] = {
        static_cast<char>(fourcc & 0xFF),
        static_cast<char>((fourcc >> 8) & 0xFF),
        static_cast<char>((fourcc >> 16) & 0xFF),
        static_cast<char>((fourcc >> 24) & 0xFF),
        '\0'
    };
    
    RCLCPP_INFO(get_logger(),
      "Camera parameters - Requested: %dx%d@%.2f, Actual: %dx%d@%.2f, Format: %s",
      frame_width_, frame_height_, fps_, actual_width, actual_height, actual_fps, pixel_format);
      
    if (actual_width != frame_width_ || actual_height != frame_height_ || 
        std::abs(actual_fps - fps_) > 0.5) {
      RCLCPP_WARN(get_logger(), "Camera parameters do not match requested values");
    }
  }

  // 设置相机信息管理器
  void setup_camera_info_manager()
  {
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    
    // 如果提供了有效的相机信息URL，则加载相机标定信息
    if (!camera_info_url_.empty() && camera_info_manager_->validateURL(camera_info_url_)) {
      if (camera_info_manager_->loadCameraInfo(camera_info_url_)) {
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
        RCLCPP_INFO(get_logger(), "Loaded camera calibration from: %s", camera_info_url_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to load camera info from: %s", camera_info_url_.c_str());
      }
    } else if (!camera_info_url_.empty()) {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url_.c_str());
    } else {
      // 创建默认的相机信息
      camera_info_msg_.width = frame_width_;
      camera_info_msg_.height = frame_height_;
      camera_info_msg_.header.frame_id = camera_name_ + "_optical_frame";
      RCLCPP_INFO(get_logger(), "Using default camera info");
    }
  }

  // 设置图像发布器
  void setup_publisher()
  {
    // 配置QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    // 创建相机图像发布器
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos.get_rmw_qos_profile());
    RCLCPP_INFO(get_logger(), "Image publisher created on topic: %s/image_raw", get_namespace());
  }

  // 设置参数回调函数
  void setup_parameter_callback()
  {
    params_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return parameters_callback(parameters);
      });
  }

  // 图像采集循环
  void capture_loop()
  {
    cv::Mat frame;
    int fail_count = 0;
    const int max_fail_count = 5;

    RCLCPP_INFO(get_logger(), "Starting image capture loop");

    while (rclcpp::ok() && running_) {
      // 读取一帧图像
      if (!read_frame(frame, fail_count)) {
        if (fail_count > max_fail_count) {
          RCLCPP_FATAL(get_logger(), "Camera failed too many times, shutting down");
          rclcpp::shutdown();
          break;
        }
        continue;
      }

      // 检查是否收到空帧
      if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty frame");
        continue;
      }

      // 发布图像帧
      publish_frame(frame);
    }
  }

  // 读取一帧图像
  bool read_frame(cv::Mat& frame, int& fail_count)
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    
    if (cap_.isOpened() && cap_.read(frame)) {
      fail_count = 0;  
      return true;
    }

    RCLCPP_WARN(get_logger(), "Failed to read frame, attempting to reopen camera");
    cap_.release();
    
    // 重新打开相机
    if (video_device_.find("/dev/") == 0){
      cap_.open(video_device_, cv::CAP_V4L2);
    } else {
      int device_id = std::stoi(video_device_);
      cap_.open(device_id, cv::CAP_V4L2);
    }
    
    // 如果重新打开成功，重新设置参数
    if (cap_.isOpened()) {
      set_camera_parameters();
    }
    
    fail_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return false;
  }

  void publish_frame(const cv::Mat& frame)
{
  try {
    std::string encoding;
    cv::Mat processed_frame;
    //先检查通道数
    RCLCPP_DEBUG(get_logger(), "Frame channels: %d, type: %d", frame.channels(), frame.type());
    if (pixel_format_ == "YUYV") {
      // 如果已经是3通道，说明OpenCV已经自动解码，直接使用
      if (frame.channels() == 3) {
        processed_frame = frame;
        encoding = "bgr8";
        RCLCPP_DEBUG(get_logger(), "YUYV already decoded to BGR, using directly");
      } else {
        // 如果是2通道，进行YUYV到BGR的转换
        cv::cvtColor(frame, processed_frame, cv::COLOR_YUV2BGR_YUY2);
        encoding = "bgr8";
        RCLCPP_DEBUG(get_logger(), "Converting YUYV to BGR");
      }
    } else if (pixel_format_ == "MJPG") {
      processed_frame = frame;
      encoding = "bgr8";
    } else {
      processed_frame = frame;
      encoding = "bgr8";
    }
    // 确保图像数据有效
    if (processed_frame.empty()) {
      RCLCPP_WARN(get_logger(), "Processed frame is empty, skipping publish");
      return;
    }
    // 创建图像消息
    auto image_msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), 
      encoding, 
      processed_frame
    ).toImageMsg();
    
    image_msg->header.stamp = now();
    image_msg->header.frame_id = camera_name_ + "_optical_frame";

    // 创建相机信息消息
    auto camera_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_);
    camera_info_msg->header = image_msg->header;

    // 发布图像和相机信息
    camera_pub_.publish(*image_msg, *camera_info_msg);
    
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error publishing frame: %s", e.what());
  }
}
  // 参数回调函数
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "frame_width") {
        frame_width_ = param.as_int();
      } else if (param.get_name() == "frame_height") {
        frame_height_ = param.as_int();
      } else if (param.get_name() == "fps") {
        fps_ = param.as_double();
      } else if (param.get_name() == "exposure") {
        exposure_ = param.as_double();
      } else if (param.get_name() == "gamma") {
        gamma_ = param.as_double();
      } else if (param.get_name() == "gain") {
        gain_ = param.as_double();
      } else if (param.get_name() == "pixel_format") {
        pixel_format_ = param.as_string();
      }
    }

    // 应用新参数
    try {
      set_camera_parameters();
      print_actual_camera_params();
    } catch (const std::exception& e) {
      result.successful = false;
      result.reason = e.what();
    }

    return result;
  }

  // 摄像头相关成员变量
  cv::VideoCapture cap_;        
  std::mutex cap_mutex_;         
  std::atomic<bool> running_{true};
  std::thread capture_thread_;   

  // 摄像头参数
  std::string video_device_;      // 摄像头设备路径或ID
  int frame_width_;               // 图像宽度
  int frame_height_;              // 图像高度
  double fps_;                    // 帧率
  double exposure_;               // 曝光值
  double gamma_;                  // Gamma值
  double gain_;                   // 增益值
  std::string pixel_format_;      // 像素格式
  std::string camera_name_;       // 相机名称
  std::string camera_info_url_;   // 相机标定文件URL

  image_transport::CameraPublisher camera_pub_; 
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_; 
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;  
};

}  // namespace usb_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera::UsbCameraNode)