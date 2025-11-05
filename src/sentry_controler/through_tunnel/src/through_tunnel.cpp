#include "through_tunnel/through_tunnel.hpp"
namespace Through_tunnel
{
    ThroughTunnel::ThroughTunnel(const rclcpp::NodeOptions & options) : Node("through_tunnel", options)
    {
        RCLCPP_INFO(this->get_logger(), "Node through_tunnel start!");

        this->buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->listener_ = std::make_unique<tf2_ros::TransformListener>(*this->buffer_);
    }
}
#include"rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Through_tunnel::ThroughTunnel);