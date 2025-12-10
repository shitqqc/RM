#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sentry_control/pid.hpp"
#include "sentry_control/sentry_mod.hpp"
#include "sentry_control/path_checker.hpp"
#include "control_interface/msg/chassis_mod.hpp"

#include "chrono"
namespace sentry_control
{
    class SentryControlNode : public rclcpp::Node
    {
        private:
        bool nav_start = false;
        bool use_spin_= false;
        double yaw_;
        double exp_spin_;
        double speed_limit_;
        
        uint32_t dt_;
        bool have_bumpy_area_;
        std::string path_topic_;
        double v_angular_max_;
        double v_angular_min_;
        double deadband;
        double kp, ki, kd;

        bumpy_area friend_bumpy_area_;
        bumpy_area enermy_bumpy_area_;

        std::shared_ptr<PID> pid_;
        std::shared_ptr<SentryMod> sentry_mod_;
        std::shared_ptr<path_checker> path_checker_;
        std::shared_ptr<control_interface::msg::ChassisMod> chassis_mod_;

        geometry_msgs::msg::Twist::SharedPtr out_cmd_vel_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_spin_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<control_interface::msg::ChassisMod>::SharedPtr chassis_mod_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        void load_params();
        void timer_callback();
        void yaw_callback(const std_msgs::msg::Float32 msg);
        void use_spin_callback(const std_msgs::msg::Bool msg);
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void speed_limit(geometry_msgs::msg::Twist::SharedPtr &cmd_vel);
        double get_aim_yaw();

    public:
        explicit SentryControlNode(const rclcpp::NodeOptions &options);
    };
} // namespace sentry_control
