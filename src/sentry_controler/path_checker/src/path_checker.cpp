#include "path_checker/path_checker.hpp"

namespace path_checker
{
    PathCheckerNode::PathCheckerNode(const rclcpp::NodeOptions &options) : rclcpp::Node("path_checker_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "PathCheckerNode is started");
        this->get_param();
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_,
            10,
            std::bind(&PathCheckerNode::path_callback, this, std::placeholders::_1));
    }

    void PathCheckerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        for (const auto &pose_stamped : msg->poses)
        {
            RCLCPP_INFO(this->get_logger(), "Received pose: [%.2f, %.2f, %.2f]",
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z);
        }
    }

    void PathCheckerNode::get_param()
    {
        this->declare_parameter<std::string>("path_topic");
        this->get_parameter("path_topic", path_topic_);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_checker::PathCheckerNode)