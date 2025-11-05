#pragma once

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace Through_tunnel
{
    class ThroughTunnel : public rclcpp::Node
    {
    private:
        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::unique_ptr<tf2_ros::TransformListener> listener_;

    public:
        ThroughTunnel(const rclcpp::NodeOptions & options);
    };
} // namespace Through_tunnel
