#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rmoss_gz_base/gz_gimbal_encoder.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace chassis_angle
{
    class ChassisAngleNode : public rclcpp::Node
    {
    private:
        bool is_sim_;
        std::string chassis_frame;
        std::string odom_frame;

        double yaw_;
        std_msgs::msg::Float32 msg;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub;

        std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

        std::shared_ptr<rmoss_gz_base::IgnGimbalEncoder> gimbal_encoder_;

        void load_param();
        void tf2_timer_callback();
        void cleanup();

    public:
        explicit ChassisAngleNode(const rclcpp::NodeOptions &options);
        ~ChassisAngleNode();
    };
} // namespace chassis_angle
