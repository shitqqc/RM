// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef BUFF_DETECTOR__DETECTOR_NODE_HPP_
#define BUFF_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "buff_interfaces/msg/blade.hpp"
#include "buff_interfaces/msg/blade_array.hpp"

#include "yolo.hpp"
#include "yolov8.hpp"
#include "thread_pool.hpp"
#include  "rm_tools/math.hpp"
#include "pnp_solver.hpp"
// STD
#include <memory>
#include <string>
#include <vector>

namespace rm_buff
{

class BuffDetectorNode : public rclcpp::Node
{
public:
  BuffDetectorNode(const rclcpp::NodeOptions & options);
  ~BuffDetectorNode();

  std::atomic<int> frame_id{0};
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void initDetector();

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();
  int detect_color;
  bool is_buff_task_;
  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);
  
  std::unique_ptr<YOLOV8> yolo_;

  cv::Mat result_img;
  
  void single_yolo_process(const cv::Mat & img, const std_msgs::msg::Header header);
  void yolo_pool_process(const cv::Mat & img, const std_msgs::msg::Header header);

  //single yolo
  std::string frame_id_;
  void single_yolo_loop();
  std::thread detect_thread_;
  std::atomic<bool> detect_thread_running_{false};
  // //thread pool
  bool use_thread_pool;
  OrderedQueue frame_queue;
  std::thread process_thread_;
  std::vector<std::unique_ptr<YOLO>> yolos;
  std::mutex yolo_mutex_;
  int num_yolo_thread;
  std::unique_ptr<ThreadPool> thread_pool_;
  std::vector<bool> yolo_used;
  void init_pool();
  void yolo_pool_loop();
  int count = 0;

  cv::Mat draw_result(const std::vector<Blade> blades, const cv::Mat &img, const float latency);
  // Detected armors publisher
  buff_interfaces::msg::BladeArray blades_msg_;
  rclcpp::Publisher<buff_interfaces::msg::BladeArray>::SharedPtr blades_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  image_transport::Publisher result_img_pub_;

};

}  // namespace rm_buff

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
