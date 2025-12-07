#pragma once
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rmoss_interfaces/msg/chassis_cmd.hpp"

namespace path_checker
{
    struct Bumpy_area
    {
        double x_max;
        double x_min;
        double y_max;
        double y_min;
    };
    class PathCheckerNode : public rclcpp::Node
    {
    private:
        bool in_bumpy_area_;
        std_msgs::msg::Bool in_bumpy_area_msg_;


        std::string path_topic_;

        Bumpy_area friend_bumpy_area_;
        Bumpy_area enermy_bumpy_area_;

        std::shared_ptr<nav_msgs::msg::Path> path_;
        std::shared_ptr<rclcpp::TimerBase> timer_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr in_bumpy_area_publisher_;

        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        // void timer_callback();

        void get_param();

        bool bumpy_area_check(double pose_x, double pose_y, Bumpy_area friend_bumpy_area, Bumpy_area enermy_bumpy_area);

    public:
        explicit PathCheckerNode(const rclcpp::NodeOptions &options);
    };
}

