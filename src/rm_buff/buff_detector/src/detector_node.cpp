// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "buff_detector/buff.hpp"
#include "buff_detector/detector_node.hpp"


namespace rm_buff
{
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions & options)
: Node("buff_detector", options)
{
  frame_id = 0;
  frame_id_ = "camera_optical_frame";
  use_thread_pool = this->declare_parameter("is_use_thread_pool",false);
  binary_thres = this->declare_parameter("binary_thres",80);
  threshold_ratio = this->declare_parameter("threshold_ratio",0.4);
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&BuffDetectorNode::imageCallback, this, std::placeholders::_1));
  if(!use_thread_pool)
  {
    initDetector();
    detect_thread_running_ = true;
    detect_thread_ = std::thread(&BuffDetectorNode::single_yolo_loop, this);
  }
  else
  {
    init_pool();
    process_thread_ = std::thread(&BuffDetectorNode::yolo_pool_loop, this);
  }
  // blades Publisher
  blades_pub_ = this->create_publisher<buff_interfaces::msg::BladeArray>(
    "/detector/blade_array", rclcpp::SensorDataQoS());
  result_img_pub_ = image_transport::create_publisher(this, "/detector/buff_result_img");

  // Task subscriber
  is_buff_task_ = false;
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_mode", 10, std::bind(&BuffDetectorNode::taskCallback, this, std::placeholders::_1));

}

BuffDetectorNode::~BuffDetectorNode()
{

  detect_thread_running_ = false;
  if (detect_thread_.joinable()) {
        detect_thread_.join();
  }
  
  if (process_thread_.joinable()) {
    process_thread_.join();
  }

}

void BuffDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
  if (task_mode == "aim" || task_mode == "auto") {
    is_buff_task_ = false;
  } else {
    is_buff_task_ = true;
  }
}

void BuffDetectorNode::single_yolo_loop()
{
  while (detect_thread_running_ && rclcpp::ok() && yolo_ != nullptr)
  {
    auto result = yolo_->debug_pop();
    auto [debug_img, blades, time] = std::move(result);
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(time);
    header.frame_id = frame_id_;
    traditional_check(blades,debug_img);
    auto t = (this->now()).seconds() * 1000.0;
    auto time_ms = time / 1e6;
    auto latency = t - time_ms;
    result_img = debug_img;
    RCLCPP_DEBUG(this->get_logger(), "detect latency: %f", latency);
    result_img = draw_result(blades, debug_img, latency);
    
    if(debug_)
      result_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", result_img).toImageMsg());

    buff_interfaces::msg::BladeArray blade_array;
    if (pnp_solver_ != nullptr) {
      blade_array.header = header;     
      // 过滤颜色
      blades.erase(
        std::remove_if(blades.begin(), blades.end(), [this](const Blade& blade) {
            return blade.color != detect_color;
        }),
        blades.end()
      );
    for(auto & blade : blades)
    {
      // solve pnp
      cv::Mat rvec, tvec;
      if (pnp_solver_->solvePnP(blade, rvec, tvec)) {
        buff_interfaces::msg::Blade blade_msg;
        blade_msg.pose.position.x = tvec.at<double>(0);
        blade_msg.pose.position.y = tvec.at<double>(1);
        blade_msg.pose.position.z = tvec.at<double>(2);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        tf2::Matrix3x3 tf_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf_quaternion;
        tf_rotation_matrix.getRotation(tf_quaternion);
        blade_msg.pose.orientation = tf2::toMsg(tf_quaternion);
        blade_msg.label = blade.color;
        blade_msg.prob = blade.prob;
        geometry_msgs::msg::Point center;
        center.x = blade.m_r.x;
        center.y = blade.m_r.y;
        blade_msg.center = center;
        blade_array.blades.emplace_back(blade_msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed");
      }
    }
      blades_pub_->publish(blade_array);
    }
  }
}

void BuffDetectorNode::yolo_pool_loop()
{
  while(rclcpp::ok()) {
    Frame process_frame;
    buff_interfaces::msg::BladeArray blade_array;
    {
    Frame process_frame = frame_queue.dequeue();
    auto img = process_frame.img;
    auto blades = process_frame.blades;
    auto t = process_frame.t;
    auto header = process_frame.header;
    auto timestamp = rclcpp::Time(t);
    traditional_check(blades,img);
    auto end = this->get_clock()->now();
    double latency = (end.seconds() - timestamp.seconds()) * 1000.0;
    result_img = draw_result(blades, img, latency);
    if(debug_) {
      result_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", result_img).toImageMsg());
    }      

    if (pnp_solver_ != nullptr) {
      blade_array.header = header;     
      // 过滤颜色
      blades.erase(
        std::remove_if(blades.begin(), blades.end(), [this](const Blade& blade) {
            return blade.color != detect_color;
        }),
        blades.end()
      );
    for(auto & blade : blades)
    {
      // solve pnp
      cv::Mat rvec, tvec;
      if (pnp_solver_->solvePnP(blade, rvec, tvec)) {
        buff_interfaces::msg::Blade blade_msg;
        blade_msg.pose.position.x = tvec.at<double>(0);
        blade_msg.pose.position.y = tvec.at<double>(1);
        blade_msg.pose.position.z = tvec.at<double>(2);

        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        tf2::Matrix3x3 tf_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf_quaternion;
        tf_rotation_matrix.getRotation(tf_quaternion);
        blade_msg.pose.orientation = tf2::toMsg(tf_quaternion);
        blade_msg.label = blade.color;
        blade_msg.prob = blade.prob;
        geometry_msgs::msg::Point center;
        center.x = blade.m_r.x;
        center.y = blade.m_r.y;
        blade_msg.center = center;
        blade_array.blades.emplace_back(blade_msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed");
      }
    }
      blades_pub_->publish(blade_array);
    }
  }
}
}

void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
  auto header = img_msg->header;
  if(!use_thread_pool && yolo_ != nullptr)
  single_yolo_process(img, header); 
  else
  yolo_pool_process(img,header);
}

void BuffDetectorNode::single_yolo_process(const cv::Mat & img, const std_msgs::msg::Header header)
{
  auto timestamp = rclcpp::Time(header.stamp);
  yolo_->push(img, timestamp.nanoseconds());
}

void BuffDetectorNode::yolo_pool_process(const cv::Mat & img, const std_msgs::msg::Header header)
{
  cv::Mat img_clone = img.clone();

  auto timestamp = rclcpp::Time(header.stamp);
  int64_t t = timestamp.nanoseconds();
  int current_frame_id = frame_id.fetch_add(1, std::memory_order_relaxed);
  //int current_frame_id = ++frame_id;

    thread_pool_->enqueue([this, img_clone = std::move(img_clone), current_frame_id, t,header] {
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
        Frame frame{current_frame_id, img_clone, t, header};
        if(!img_clone.empty())
        {frame.blades = yolos[yolo_id]->detect(frame.img);
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
void BuffDetectorNode::initDetector()
{
    auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");    
    std::string model_name = this->declare_parameter("model_name", "best.xml");
    auto model_path = pkg_path + "/models/" + model_name;  
    std::string device = declare_parameter("device","CPU");
    auto conf_threshold = this->declare_parameter("confidence_threshole", 0.5);
    yolo_ = nullptr;
    yolo_ = std::make_unique<YOLOV8>(model_path, conf_threshold, device);
    detect_color = declare_parameter("detect_color", 0);//0:red 1:blue
    RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());

}

void BuffDetectorNode::init_pool()
{
    auto pkg_path = ament_index_cpp::get_package_share_directory("buff_detector");    
    std::string model_name = this->declare_parameter("model_name", "best.xml");
    auto model_path = pkg_path + "/models/" + model_name;  
    std::string device = this->declare_parameter("device","CPU");
    num_yolo_thread = this->declare_parameter("num_yolo_thread",8);
    auto conf_threshold = this->declare_parameter("confidence_threshole", 0.7);
    yolos = create_yolov8s(model_path, num_yolo_thread, conf_threshold, device);
    yolo_used.resize(num_yolo_thread, false);
    thread_pool_ = std::make_unique<ThreadPool>(num_yolo_thread);
    detect_color = declare_parameter("detect_color", 0);//0:red 1:blue
    RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_path.c_str());

}
cv::Mat BuffDetectorNode::draw_result(const std::vector<Blade> blades, const cv::Mat &img, const float latency)
{
   cv::Mat result_img_ = img.clone();
    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      result_img_, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

  for(auto & blade : blades)
  {
    std::stringstream result_ss;
    result_ss << std::fixed << std::setprecision(1)<< blade.prob * 100.0 << "%";
    auto result_s = result_ss.str();
    cv::putText(
      result_img_, result_s, blade.m_top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
    cv::circle(result_img_, blade.m_top, 5, cv::Scalar(0, 0, 255), -1);  // 红色，半径为5
    cv::circle(result_img_, blade.m_bottom, 5, cv::Scalar(0, 255, 0), -1);  // 绿色，半径为5
    cv::circle(result_img_, blade.m_left, 5, cv::Scalar(255, 0, 0), -1);  // 蓝色，半径为5
    cv::circle(result_img_, blade.m_right, 5, cv::Scalar(0, 255, 255), -1);  // 黄色，半径为5
    cv::circle(result_img_, blade.m_r, 5, cv::Scalar(255, 0, 255), -1);  // 紫色，半径为5
  }
  return result_img_;
}

void BuffDetectorNode::traditional_check(std::vector<Blade>& blades, const cv::Mat& img)
{

    blades.erase(
        std::remove_if(blades.begin(), blades.end(), [&](Blade& blade) {
            // 计算矩形的左上角和右下角
            cv::Rect blade_rect(
                cv::Point(std::min({blade.m_top.x, blade.m_bottom.x, blade.m_left.x, blade.m_right.x}),
                          std::min({blade.m_top.y, blade.m_bottom.y, blade.m_left.y, blade.m_right.y})),
                cv::Point(std::max({blade.m_top.x, blade.m_bottom.x, blade.m_left.x, blade.m_right.x}),
                          std::max({blade.m_top.y, blade.m_bottom.y, blade.m_left.y, blade.m_right.y}))
            );

            if (blade_rect.x < 0 || blade_rect.y < 0 || blade_rect.x + blade_rect.width >= img.cols || blade_rect.y + blade_rect.height >= img.rows)
                return true;

            // 提取矩形区域
            cv::Mat region = img(blade_rect);

            // 二值化
            cv::Mat binary;
            cv::threshold(region, binary, binary_thres, 255, cv::THRESH_BINARY);

            // 计算白色像素的占比
            double white_pixel_count = cv::countNonZero(binary);  // 计算非零（白色）像素数
            double area = blade_rect.area();
            double white_ratio = white_pixel_count / area;

            // 如果白色像素占比小于阈值，则过滤掉该矩形
            return white_ratio < threshold_ratio;
        }),
        blades.end()
    );
}
}  // namespace rm_buff

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_buff::BuffDetectorNode)
