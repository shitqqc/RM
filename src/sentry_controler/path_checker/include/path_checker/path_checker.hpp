#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"

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
        std::string path_topic_;

        Bumpy_area friend_bumpy_area_;
        Bumpy_area enermy_bumpy_area_;

        std::shared_ptr<nav_msgs::msg::Path> path_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;

        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

        void get_param();

    public:
        explicit PathCheckerNode(const rclcpp::NodeOptions &options);
    };
}

