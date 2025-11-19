#include "path_checker/path_checker.hpp"

namespace path_checker
{
    PathCheckerNode::PathCheckerNode(const rclcpp::NodeOptions &options) : rclcpp::Node("path_checker_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "PathCheckerNode is started");
        this->get_param();
        RCLCPP_INFO(this->get_logger(), "friend bumpy area x:[%.2lf, %.2lf], y:[%.2lf, %.2lf]", friend_bumpy_area_.x_min, friend_bumpy_area_.x_max, friend_bumpy_area_.y_min, friend_bumpy_area_.y_max);
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_,
            10,
            std::bind(&PathCheckerNode::path_callback, this, std::placeholders::_1));
    }
    void PathCheckerNode::get_param()
    {
        this->declare_parameter<std::string>("path_topic");
        this->declare_parameter<double>("friend_bumpy_area.x_max", 5.0);
        this->declare_parameter<double>("friend_bumpy_area.x_min", -5.0);  
        this->declare_parameter<double>("friend_bumpy_area.y_max", 5.0);
        this->declare_parameter<double>("friend_bumpy_area.y_min", -5.0);
        this->declare_parameter<double>("enermy_bumpy_area.x_max", 5.0);
        this->declare_parameter<double>("enermy_bumpy_area.x_min", -5.0);  
        this->declare_parameter<double>("enermy_bumpy_area.y_max", 5.0);
        this->declare_parameter<double>("enermy_bumpy_area.y_min", -5.0);

        this->get_parameter("path_topic", path_topic_);
        this->get_parameter("friend_bumpy_area.x_max", friend_bumpy_area_.x_max);
        this->get_parameter("friend_bumpy_area.x_min", friend_bumpy_area_.x_min);
        this->get_parameter("friend_bumpy_area.y_max", friend_bumpy_area_.y_max);
        this->get_parameter("friend_bumpy_area.y_min", friend_bumpy_area_.y_min);
        this->get_parameter("enermy_bumpy_area.x_max", enermy_bumpy_area_.x_max);
        this->get_parameter("enermy_bumpy_area.x_min", enermy_bumpy_area_.x_min);
        this->get_parameter("enermy_bumpy_area.y_max", enermy_bumpy_area_.y_max);
        this->get_parameter("enermy_bumpy_area.y_min", enermy_bumpy_area_.y_min);
    }

    void PathCheckerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        bool in_bumpy_area = false;
        for (const auto &pose_stamped : msg->poses)
        {
            if((pose_stamped.pose.position.x <= friend_bumpy_area_.x_max &&
                pose_stamped.pose.position.x >= friend_bumpy_area_.x_min &&
                pose_stamped.pose.position.y <= friend_bumpy_area_.y_max &&
                pose_stamped.pose.position.y >= friend_bumpy_area_.y_min)||
            (
                pose_stamped.pose.position.x <= enermy_bumpy_area_.x_max &&
                pose_stamped.pose.position.x >= enermy_bumpy_area_.x_min &&
                pose_stamped.pose.position.y <= enermy_bumpy_area_.y_max &&
                pose_stamped.pose.position.y >= enermy_bumpy_area_.y_min
            ))
            {
                in_bumpy_area = true;
                break;;
            }
        }
        if(in_bumpy_area)
        {
            RCLCPP_WARN(this->get_logger(), "Path is in the friend bumpy area!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Path is safe from the friend bumpy area.");
        }
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_checker::PathCheckerNode)