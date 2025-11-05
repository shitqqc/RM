#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sentry_control/pid.hpp"

#include "chrono"
namespace sentry_control
{
    class SentryControlNode : public rclcpp::Node
    {
        private:
        uint32_t dt_;
        double yaw_;
        double exp_yaw_;
        double exp_spin_;
        bool nav_start = false;

        double v_angular_max_;
        double v_angular_min_;
        double kp, ki, kd;

        std::shared_ptr<PID> pid_;

        geometry_msgs::msg::Twist out_cmd_vel_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        void load_params();
        void timer_callback();
        void cmd_vel_callback(const geometry_msgs::msg::Twist msg);
        void handle_yaw_message(const std_msgs::msg::Float32::SharedPtr msg);
        
    public:
        explicit SentryControlNode(const rclcpp::NodeOptions &options);
    };
} // namespace sentry_control
