#include "test_publisher/test_publisher.hpp"

namespace test_publisher
{
    testPublisher::testPublisher(const rclcpp::NodeOptions &options) : Node("test_publisher", options)
    {
        this->test_publisher_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&testPublisher::test_publisher_timer_callback, this));
    }
    void testPublisher::test_publisher_timer_callback()
    {
        test_msg_ = std::make_shared<geometry_msgs::msg::Twist>();
        test_msg_->linear.x = 1.0;
        test_msg_->angular.z = 0.5;

        test_publisher_pub->publish(*test_msg_);
        RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x=%.2f, angular.z=%.2f",
                    test_msg_->linear.x, test_msg_->angular.z);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(test_publisher::testPublisher)