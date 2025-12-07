#include "path_checker/path_checker.hpp"

namespace path_checker
{
    PathCheckerNode::PathCheckerNode(const rclcpp::NodeOptions &options) : rclcpp::Node("path_checker_node", options)
    {
        RCLCPP_INFO(this->get_logger(), "PathCheckerNode is started");
        this->get_param();
        in_bumpy_area_ = false;
        // RCLCPP_INFO(this->get_logger(), "friend bumpy area x:[%.2lf, %.2lf], y:[%.2lf, %.2lf]", friend_bumpy_area_.x_min, friend_bumpy_area_.x_max, friend_bumpy_area_.y_min, friend_bumpy_area_.y_max);
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_,
            10,
            std::bind(&PathCheckerNode::path_callback, this, std::placeholders::_1));
        in_bumpy_area_publisher_ = this->create_publisher<std_msgs::msg::Bool>("in_bumpy_area", 10);
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100),
        //     std::bind(&PathCheckerNode::timer_callback, this));
    }
    void PathCheckerNode::get_param()
    {
        this->declare_parameter<std::string>("path_topic", "path");
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

    // void PathCheckerNode::timer_callback()
    // {

    // }

    void PathCheckerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.size() < 10)
        {
            for (size_t i = 0; i < msg->poses.size(); i++)
            {
                in_bumpy_area_ = bumpy_area_check(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, friend_bumpy_area_, enermy_bumpy_area_);
                if (in_bumpy_area_) break;
            }
        }
        else
        {
            for (size_t i = 0; i < 10; i++)
            {
                in_bumpy_area_ = bumpy_area_check(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, friend_bumpy_area_, enermy_bumpy_area_);
                if (in_bumpy_area_) break;
            }
        }
        in_bumpy_area_msg_.data = in_bumpy_area_;
        in_bumpy_area_publisher_->publish(in_bumpy_area_msg_);
    }

    bool PathCheckerNode::bumpy_area_check(double pose_x, double pose_y, Bumpy_area friend_bumpy_area, Bumpy_area enermy_bumpy_area)
    {
        return (pose_x <= friend_bumpy_area.x_max &&
                pose_x >= friend_bumpy_area.x_min &&
                pose_y <= friend_bumpy_area.y_max &&
                pose_y >= friend_bumpy_area.y_min) ||
               (pose_x <= enermy_bumpy_area.x_max &&
                pose_x >= enermy_bumpy_area.x_min &&
                pose_y <= enermy_bumpy_area.y_max &&
                pose_y >= enermy_bumpy_area.y_min);
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_checker::PathCheckerNode)