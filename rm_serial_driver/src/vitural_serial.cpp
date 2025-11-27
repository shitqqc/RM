// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.

// std
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>

#include <rclcpp/executors.hpp>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_serial_driver {
class VirtualSerialNode : public rclcpp::Node {

public:
  explicit VirtualSerialNode(const rclcpp::NodeOptions &options) : Node("serial_driver", options) {
    RCLCPP_INFO(this->get_logger(), "Starting VirtualSerialNode!");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "gimbal_link";

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
    //   double roll = this->get_parameter("roll").as_double();
    //   double pitch = this->get_parameter("pitch").as_double();
    //   double yaw = this->get_parameter("yaw").as_double();
    double roll= 0;
    double pitch = 0;
    double yaw = 0;
      tf2::Quaternion q;
      q.setRPY(roll * M_PI / 180.0, -pitch * M_PI / 180.0, yaw * M_PI / 180.0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom";
      transform_stamped_.child_frame_id = "gimbal_link";
      transform_stamped_.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_stamped_);

    //   Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
    //   Eigen::Vector3d rpy  = utils::getRPY(q_eigen.toRotationMatrix());
    //   q.setRPY(rpy[0], 0, 0);
    //   transform_stamped_.transform.rotation = tf2::toMsg(q);
    //   transform_stamped_.header.frame_id = "odom";
    //   transform_stamped_.child_frame_id = "odom_rectify";
    //   tf_broadcaster_->sendTransform(transform_stamped_);

    });
  }
private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
};
}  // namespace fyt::serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::VirtualSerialNode)
