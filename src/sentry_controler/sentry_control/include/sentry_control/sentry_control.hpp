#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sentry_control/pid.hpp"
#include "control_interface/msg/chassis_mod.hpp"

#include "chrono"
namespace sentry_control
{
    class SentryControlNode : public rclcpp::Node
    {
        private:
        uint32_t dt_;
        double yaw_;
        double exp_spin_;
        bool nav_start = false;

        double v_angular_max_;
        double v_angular_min_;
        double deadband;
        double kp, ki, kd;

        std::shared_ptr<PID> pid_;
        std::shared_ptr<control_interface::msg::ChassisMod> chassis_mod_;

        geometry_msgs::msg::Twist::SharedPtr out_cmd_vel_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<control_interface::msg::ChassisMod>::SharedPtr chassis_mod_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        void load_params();
        void timer_callback();
        void cmd_vel_callback(const geometry_msgs::msg::Twist msg);
        void chassis_mod_callback(const control_interface::msg::ChassisMod msg);
        
    public:
        explicit SentryControlNode(const rclcpp::NodeOptions &options);
    };
} // namespace sentry_control
