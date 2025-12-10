// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

#include "armor_detector/detector.hpp"
#include "armor_detector/armor_pose_estimator.hpp"
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
#include <visualization_msgs/msg/marker_array.hpp>
#include "auto_aim_interfaces/msg/armor.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <tf2_ros/create_timer_ros.h>

#include "yolo.hpp"
#include "yolov5.hpp"
#include "thread_pool.hpp"
// STD
#include <memory>
#include <string>
#include <vector>

namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);
  ~ArmorDetectorNode();

  std::atomic<int> frame_id{0};
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void initDetector();

  void createDebugPublishers();
  void destroyDebugPublishers();

  void publishMarkers();
  int detect_color;
  bool use_traditional;
  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  bool is_aim_task_;
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);
  // Armor Detector
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<YOLOV5> yolo_;

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

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;

  // ReceiveData subscripiton
  std::string odom_frame_;
  Eigen::Matrix3d imu_to_camera_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
