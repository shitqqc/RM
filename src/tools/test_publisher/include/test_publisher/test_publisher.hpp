#pragma once

#include "chrono"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace test_publisher

{
    class testPublisher : public rclcpp::Node
    {
    private:
        geometry_msgs::msg::Twist::SharedPtr test_msg_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr test_publisher_pub;

        void test_publisher_timer_callback();
    public:
        explicit testPublisher(const rclcpp::NodeOptions &options);
    };
} // namespace test_publisher
