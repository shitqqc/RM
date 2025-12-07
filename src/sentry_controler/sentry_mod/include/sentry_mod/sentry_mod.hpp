#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control_interface/msg/chassis_mod.hpp"

namespace sentry_mod
{
    class SentryMod : public rclcpp::Node
    {
    private:
        bool in_bumpy_area_;
        bool use_spin_;

        double default_spin_speed_;

        control_interface::msg::ChassisMod::SharedPtr chassis_mod_msg_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr in_bumpy_area_sub_;
        rclcpp::Publisher<control_interface::msg::ChassisMod>::SharedPtr chassis_mod_pub_; 
        rclcpp::TimerBase::SharedPtr timer_;

        void load_params();
        void in_bumpy_area_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void timer_callback();
    public:
        explicit SentryMod(const rclcpp::NodeOptions &options);
    };    
} // namespace sentry_mod
