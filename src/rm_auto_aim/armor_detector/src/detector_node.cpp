// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2/time.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"
#include"rm_tools/math.hpp"

namespace rm_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  frame_id = 0;
  frame_id_ = "camera_optical_frame";
  //use_thread_pool = this->declare_parameter("is_use_thread_pool",false);
  detect_mode = this->declare_parameter("detect_mode",0);
  
  // 是否启用姿态估计（默认启用）
  bool enable_pose_estimation = this->declare_parameter("enable_pose_estimation", true);
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode! Pose estimation: %s", 
              enable_pose_estimation ? "enabled" : "disabled");

  // 只有启用姿态估计时才订阅camera_info
  if (enable_pose_estimation) {
    odom_frame_ = this->declare_parameter("target_frame", "odom");
    imu_to_camera_ = Eigen::Matrix3d::Identity();
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>(cam_info_);
        armor_pose_estimator_->use_ba_ = this->declare_parameter("use_ba",true);
        cam_info_sub_.reset();
      });
  } else {
    // 禁用姿态估计时，只获取相机中心点
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        // 不创建armor_pose_estimator_，只做检测和分类
        cam_info_sub_.reset();
      });
  }
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
  if(detect_mode != 2)
  {
    initDetector();
  }
  if(detect_mode == 1)
  {
    detect_thread_running_ = true;
    detect_thread_ = std::thread(&ArmorDetectorNode::async_yolo_loop, this);
  }
  else if(detect_mode == 2)
  {
    init_pool();
    process_thread_ = std::thread(&ArmorDetectorNode::yolo_pool_loop, this);
  }

  // Armors Publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());



  // Task subscriber
  is_aim_task_ = true;
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_mode", 10, std::bind(&ArmorDetectorNode::taskCallback, this, std::placeholders::_1));
  
  // Visualization Marker Publisher
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });
  
  // tf2_buffer_ 已在上面根据enable_pose_estimation条件初始化
  // 如果启用姿态估计，需要设置timer接口
  if (enable_pose_estimation && tf2_buffer_) {
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
  }

}

ArmorDetectorNode::~ArmorDetectorNode()
{

  detect_thread_running_ = false;
  if (detect_thread_.joinable()) {
        detect_thread_.join();
  }
  
  if (process_thread_.joinable()) {
    process_thread_.join();
  }

}

void ArmorDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
  if (task_mode == "aim" || task_mode == "auto") {
    is_aim_task_ = true;
  } else {
    is_aim_task_ = false;
  }
}

void ArmorDetectorNode::async_yolo_loop()
{
  while (detect_thread_running_ && rclcpp::ok() && yolo_ != nullptr)
  {
    auto result = yolo_->debug_pop();
    auto [debug_img, armors, time] = std::move(result);
    if(use_traditional)
    {
      for(auto & armor : armors)
      {
        detector_->traditional_correct(armor,debug_img);
      }
    }
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(time);
    header.frame_id = frame_id_;
    auto t = (this->now()).seconds() * 1000.0;
    auto time_ms = time / 1e6;
    auto latency = t - time_ms;
    
    RCLCPP_DEBUG(this->get_logger(), "detect latency: %f", latency);

    if (detector_ == nullptr) continue;
    auto t1 = (this->now()).seconds() * 1000.0;
    detector_->result_img = detector_->draw_result(armors, debug_img, latency);
      auto t2 = (this->now()).seconds() * 1000.0;
      auto lll = t2 - t1;
      RCLCPP_DEBUG(this->get_logger(), "pnp latency: %f", lll);
    
    if(debug_)
    {
      result_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", detector_->result_img).toImageMsg());
      auto all_binary_img = detector_->get_all_binary_img(armors);
      binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
    }

    if (armor_pose_estimator_ != nullptr) {
      armors_msg_.header = header;
      
      // 过滤颜色
      armors.erase(
        std::remove_if(armors.begin(), armors.end(), [this](const Armor& armor) {
            return static_cast<int>(armor.color) != detect_color;
        }),
        armors.end()
      );
      bool tf_available = false;
      try {
        // 使用 tf2::TimePointZero 获取最新的可用变换，避免时间外推错误
        auto odom_to_gimbal = tf2_buffer_->lookupTransform(
            odom_frame_, header.frame_id, tf2::TimePointZero);
            
        auto msg_q = odom_to_gimbal.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix(tf_q);

        imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
                          tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
                          tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
        tf_available = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TF lookup failed (odom->%s): %s. Skipping pose estimation.", header.frame_id.c_str(), ex.what());
        // TF 不可用时，跳过姿态估计，但仍发布检测结果
        armors_msg_.armors.clear();
        for (const auto& armor : armors) {
          auto_aim_interfaces::msg::Armor armor_msg;
          armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
          armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
          armor_msg.color = COLORS[static_cast<int>(armor.color)];
          // 不设置姿态信息，只发布检测结果
          armors_msg_.armors.push_back(armor_msg);
        }
        armors_pub_->publish(armors_msg_);
        return;
      }
      if (tf_available) {
        armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
      }

      if (debug_) {
          marker_array_.markers.clear();
          armor_marker_.id = 0;
          text_marker_.id = 0;
          armor_marker_.header = text_marker_.header = armors_msg_.header;
          for (const auto &armor : armors_msg_.armors) {
            armor_marker_.scale.y = armor.type == std::string("small") ? 0.135 : 0.23;
            armor_marker_.pose = armor.pose;
            armor_marker_.id++;
            text_marker_.pose.position = armor.pose.position;
            text_marker_.id++;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = armor.number;
            marker_array_.markers.emplace_back(armor_marker_);
            marker_array_.markers.emplace_back(text_marker_);
          }
          publishMarkers();
      }

      armors_pub_->publish(armors_msg_);
    } else {
      // 无姿态估计模式：只发布检测和分类结果（用于USB相机）
      armors_msg_.header = header;
      
      // 过滤颜色
      armors.erase(
        std::remove_if(armors.begin(), armors.end(), [this](const Armor& armor) {
            return static_cast<int>(armor.color) != detect_color;
        }),
        armors.end()
      );
      
      // 转换为消息格式（不包含姿态信息）
      armors_msg_.armors.clear();
      for (const auto &armor : armors) {
        auto_aim_interfaces::msg::Armor armor_msg;
        
        // 填充基本信息（检测和分类）
        armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
        armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
        armor_msg.color = COLORS[static_cast<int>(armor.color)];
        armor_msg.prob = armor.confidence;
        armor_msg.yaw_raw = armor.yaw_raw;
        
        // 姿态信息设为默认值（不使用）
        armor_msg.pose.position.x = 0.0;
        armor_msg.pose.position.y = 0.0;
        armor_msg.pose.position.z = 0.0;
        armor_msg.pose.orientation.x = 0.0;
        armor_msg.pose.orientation.y = 0.0;
        armor_msg.pose.orientation.z = 0.0;
        armor_msg.pose.orientation.w = 1.0;
        armor_msg.yaw = 0.0;
        armor_msg.pitch = 0.0;
        armor_msg.roll = 0.0;
        armor_msg.dyaw = 0.0;
        
        // 计算到图像中心的距离
        if (cam_info_) {
          cv::Point2f image_center(cam_info_->k[2], cam_info_->k[5]);
          armor_msg.distance_to_image_center = 
              cv::norm(armor.center - image_center);
        } else {
          armor_msg.distance_to_image_center = 0.0;
        }
        
        // 检测框的四个角点
        armor_msg.kpts.clear();
        for (const auto &pt : armor.points) {
          geometry_msgs::msg::Point point;
          point.x = pt.x;
          point.y = pt.y;
          point.z = 0.0;
          armor_msg.kpts.emplace_back(point);
        }
        
        armor_msg.priority = static_cast<int>(armor.priority);
        
        armors_msg_.armors.emplace_back(std::move(armor_msg));
      }
      
      // 发布检测结果（仅包含检测和分类，无姿态）
      armors_pub_->publish(armors_msg_);
    }
  }
}

void ArmorDetectorNode::yolo_pool_loop()
{
  while(rclcpp::ok()) {
    Frame process_frame;
    {
    Frame process_frame = frame_queue.dequeue();
    auto img = process_frame.img;
    auto armors = process_frame.armors;
    auto t = process_frame.t;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(t);
    header.frame_id = frame_id_;
    if(use_traditional)
    {
      for(auto & armor : armors)
        detector_->traditional_correct(armor, img);
    }
    auto timestamp = rclcpp::Time(t);
    auto end = this->get_clock()->now();
    double latency = (end.seconds() - timestamp.seconds()) * 1000.0;
    
    if(debug_) {
      detector_->result_img = detector_->draw_result(armors, img, latency);
      result_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", detector_->result_img).toImageMsg());
      auto all_binary_img = detector_->get_all_binary_img(armors);
      binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
    }  
    
    if (armor_pose_estimator_ != nullptr) {
      armors_msg_.header = header;
      armors.erase(
        std::remove_if(armors.begin(), armors.end(), [this](const Armor& armor) {
            return static_cast<int>(armor.color) != detect_color;}),
            armors.end()
      );
      
      bool tf_available = false;
      try {
        rclcpp::Time target_time = header.stamp;
        // 使用 tf2::TimePointZero 获取最新的可用变换，避免时间外推错误
        auto odom_to_gimbal = tf2_buffer_->lookupTransform(
            odom_frame_, header.frame_id, tf2::TimePointZero);
        auto msg_q = odom_to_gimbal.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
        imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
            tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0],
            tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
            tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
            tf2_matrix.getRow(2)[2];
        tf_available = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TF lookup failed (odom->%s): %s. Skipping pose estimation.", header.frame_id.c_str(), ex.what());
        // TF 不可用时，跳过姿态估计，但仍发布检测结果
        armors_msg_.armors.clear();
        for (const auto& armor : armors) {
          auto_aim_interfaces::msg::Armor armor_msg;
          armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
          armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
          armor_msg.color = COLORS[static_cast<int>(armor.color)];
          // 不设置姿态信息，只发布检测结果
          armors_msg_.armors.push_back(armor_msg);
        }
        armors_pub_->publish(armors_msg_);
        return;
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TF lookup error (odom->%s): %s. Skipping pose estimation.", header.frame_id.c_str(), ex.what());
        // TF 不可用时，跳过姿态估计，但仍发布检测结果
        armors_msg_.armors.clear();
        for (const auto& armor : armors) {
          auto_aim_interfaces::msg::Armor armor_msg;
          armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
          armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
          armor_msg.color = COLORS[static_cast<int>(armor.color)];
          // 不设置姿态信息，只发布检测结果
          armors_msg_.armors.push_back(armor_msg);
        }
        armors_pub_->publish(armors_msg_);
        return;
      }
      auto x1 = std::chrono::high_resolution_clock::now();
      if (tf_available) {
        armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
      }
      auto x2 = std::chrono::high_resolution_clock::now();
      auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(x2 -x1);
      //std::cout<<"armor_pose_estimator_latency:"<<total_time.count()/1000.0<<std::endl;

      if (debug_) {
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;
        armor_marker_.header = text_marker_.header = armors_msg_.header;
        
        for (const auto &armor : armors_msg_.armors) {
          armor_marker_.scale.y = armor.type == std::string("small") ? 0.135 : 0.23;
          armor_marker_.pose = armor.pose;
          armor_marker_.id++;
          text_marker_.pose.position = armor.pose.position;
          text_marker_.id++;
          text_marker_.pose.position.y -= 0.1;
          text_marker_.text = armor.number;
          marker_array_.markers.emplace_back(armor_marker_);
          marker_array_.markers.emplace_back(text_marker_);
        }
        publishMarkers();
      }
      
      armors_pub_->publish(armors_msg_);
    }
  }
}
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
  auto timestamp = rclcpp::Time(img_msg->header.stamp);
  int64_t t = timestamp.nanoseconds();
  if(detect_mode == 0)
  single_yolo_process(img, t);
  else if(detect_mode == 1 && yolo_ != nullptr)
  yolo_->push(img, timestamp.nanoseconds());
  else
  yolo_pool_process(img,t);
}

void ArmorDetectorNode::single_yolo_process(const cv::Mat & img, int64_t t)
{
  std::vector<Armor> armors;
  armors = yolo_->detect(img);
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(t);
    header.frame_id = frame_id_;
    if(use_traditional)
    {
      for(auto & armor : armors)
        detector_->traditional_correct(armor, img);
    }
    auto timestamp = rclcpp::Time(t);
    auto end = this->get_clock()->now();
    double latency = (end.seconds() - timestamp.seconds()) * 1000.0;
    
    if(debug_) {
      detector_->result_img = detector_->draw_result(armors, img, latency);
      result_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", detector_->result_img).toImageMsg());
      auto all_binary_img = detector_->get_all_binary_img(armors);
      binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_binary_img).toImageMsg());
    }  
    
    if (armor_pose_estimator_ != nullptr) {
      armors_msg_.header = header;
      // armors.erase(
      //   std::remove_if(armors.begin(), armors.end(), [this](const Armor& armor) {
      //       return static_cast<int>(armor.color) != detect_color;}),
      //       armors.end()
      // );
      
      bool tf_available = false;
      try {
        rclcpp::Time target_time = header.stamp;
        // 使用 tf2::TimePointZero 获取最新的可用变换，避免时间外推错误
        auto odom_to_gimbal = tf2_buffer_->lookupTransform(
            odom_frame_, header.frame_id, tf2::TimePointZero);
        auto msg_q = odom_to_gimbal.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
        imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
            tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0],
            tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
            tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
            tf2_matrix.getRow(2)[2];
        tf_available = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TF lookup failed (odom->%s): %s. Skipping pose estimation.", header.frame_id.c_str(), ex.what());
        // TF 不可用时，跳过姿态估计，但仍发布检测结果
        armors_msg_.armors.clear();
        for (const auto& armor : armors) {
          auto_aim_interfaces::msg::Armor armor_msg;
          armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
          armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
          armor_msg.color = COLORS[static_cast<int>(armor.color)];
          // 不设置姿态信息，只发布检测结果
          armors_msg_.armors.push_back(armor_msg);
        }
        armors_pub_->publish(armors_msg_);
        return;
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "TF lookup error (odom->%s): %s. Skipping pose estimation.", header.frame_id.c_str(), ex.what());
        // TF 不可用时，跳过姿态估计，但仍发布检测结果
        armors_msg_.armors.clear();
        for (const auto& armor : armors) {
          auto_aim_interfaces::msg::Armor armor_msg;
          armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
          armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
          armor_msg.color = COLORS[static_cast<int>(armor.color)];
          // 不设置姿态信息，只发布检测结果
          armors_msg_.armors.push_back(armor_msg);
        }
        armors_pub_->publish(armors_msg_);
        return;
      }
      auto x1 = std::chrono::high_resolution_clock::now();
      if (tf_available) {
        armors_msg_.armors = armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
      }
      auto x2 = std::chrono::high_resolution_clock::now();
      auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(x2 -x1);
      //std::cout<<"armor_pose_estimator_latency:"<<total_time.count()/1000.0<<std::endl;

      if (debug_) {
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;
        armor_marker_.header = text_marker_.header = armors_msg_.header;
        
        for (const auto &armor : armors_msg_.armors) {
          armor_marker_.scale.y = armor.type == std::string("small") ? 0.135 : 0.23;
          armor_marker_.pose = armor.pose;
          armor_marker_.id++;
          text_marker_.pose.position = armor.pose.position;
          text_marker_.id++;
          text_marker_.pose.position.y -= 0.1;
          text_marker_.text = armor.number;
          marker_array_.markers.emplace_back(armor_marker_);
          marker_array_.markers.emplace_back(text_marker_);
        }
        publishMarkers();
      }
      
      armors_pub_->publish(armors_msg_);
    }
}
void ArmorDetectorNode::yolo_pool_process(const cv::Mat & img, int64_t t)
{
  cv::Mat img_clone = img.clone();
  int current_frame_id = frame_id.fetch_add(1, std::memory_order_relaxed);
  //int current_frame_id = ++frame_id;

    thread_pool_->enqueue([this, img_clone = std::move(img_clone), current_frame_id, t] {
      int yolo_id = -1;
      {
        std::lock_guard<std::mutex> lock(yolo_mutex_);
        for (int i = 0; i < num_yolo_thread; i++) {
          if (!yolo_used[i]) {
            yolo_used[i] = true;
            yolo_id = i;
            break;
          }
        }
      }
      if (yolo_id != -1) {  
        Frame frame{current_frame_id, img_clone, t};
        if(!img_clone.empty())
        {frame.armors = yolos[yolo_id]->detect(frame.img);
        frame_queue.enqueue(frame);}
      else
      std::cout<<"the img is fucking empty"<<std::endl;
      {
        std::lock_guard<std::mutex> lock(yolo_mutex_);
        yolo_used[yolo_id] = false;
      }  
      }
      else
      RCLCPP_WARN(this->get_logger(), "YOLO Pool is full!");
    });
}
void ArmorDetectorNode::initDetector()
{
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");    
    std::string model_name = this->declare_parameter("model_name", "yolov5.xml");
    auto model_path = pkg_path + "/models/" + model_name;  
    std::string device = declare_parameter("device","CPU");
    auto conf_threshold = this->declare_parameter("confidence_threshole", 0.5);
    yolo_ = nullptr;
    yolo_ = std::make_unique<YOLOV5>(model_path, conf_threshold, device, LATENCY_MODE);
    detector_ = std::make_unique<Detector>();
    detect_color = declare_parameter("detect_color", 0);//0:red 1:blue
    detector_-> binary_thres = this->declare_parameter("binary_threshold", 70);
    use_traditional = this->declare_parameter("use_traditional", true);
    detector_-> max_angle_error_ = this->declare_parameter("max_angle_error", 50);
    detector_-> min_lightbar_ratio_ = this->declare_parameter("min_lightbar_ratio", 1.5);
    detector_-> max_lightbar_ratio_ = this->declare_parameter("max_lightbar_ratio", 20);
    detector_-> min_lightbar_length_ = this->declare_parameter("min_lightbar_length", 20);
    detector_-> tolerate = this->declare_parameter("tolerate", 0.1);
    detector_-> min_l2l_ratio_ = this->declare_parameter("min_l2l_ratio", 0.7);
    detector_->use_pca = this->declare_parameter("use_pca",false);
    //if(armor_pose_estimator_ != nullptr)

    RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());

}

void ArmorDetectorNode::init_pool()
{
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");    
    std::string model_name = this->declare_parameter("model_name", "yolov5.xml");
    auto model_path = pkg_path + "/models/" + model_name;  
    std::string device = this->declare_parameter("device","CPU");
    num_yolo_thread = this->declare_parameter("num_yolo_thread",8);
    auto conf_threshold = this->declare_parameter("confidence_threshole", 0.7);
    yolos = create_yolov5s(model_path, num_yolo_thread, conf_threshold, device);
    yolo_used.resize(num_yolo_thread, false);
    thread_pool_ = std::make_unique<ThreadPool>(num_yolo_thread);
    detector_ = std::make_unique<Detector>();
    detect_color = declare_parameter("detect_color", 0);//0:red 1:blue
    detector_-> binary_thres = this->declare_parameter("binary_threshold", 70);
    detector_-> max_angle_error_ = this->declare_parameter("max_angle_error", 50);
    detector_-> min_lightbar_ratio_ = this->declare_parameter("min_lightbar_ratio", 1.5);
    detector_-> max_lightbar_ratio_ = this->declare_parameter("max_lightbar_ratio", 20);
    detector_-> min_lightbar_length_ = this->declare_parameter("min_lightbar_length", 20);
    detector_-> tolerate = this->declare_parameter("tolerate", 0.1);
    detector_-> min_l2l_ratio_ = this->declare_parameter("min_l2l_ratio", 0.7);
    detector_->use_pca = this->declare_parameter("use_pca",false);
    RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());

}

void ArmorDetectorNode::createDebugPublishers()
{
  this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
  this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers()
{
  binary_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
