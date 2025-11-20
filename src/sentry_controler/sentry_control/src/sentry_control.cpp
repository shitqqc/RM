#include "sentry_control/sentry_control.hpp"

/* 
todo: 
    ok啊，还得是别人的pid好用
*/

namespace sentry_control
{
    SentryControlNode::SentryControlNode(const rclcpp::NodeOptions &options) : Node("sentry_control", options)
    {
        RCLCPP_INFO(this->get_logger(), "Sentry control node start");

        this->load_params();
        RCLCPP_INFO(this->get_logger(), "Parameters loaded: dt=%d, max=%f, min=%f, kp=%f, kd=%f, ki=%f", dt_, v_angular_max_, v_angular_min_, kp, kd, ki);

        pid_ = std::make_shared<PID>(dt_/1000.0, v_angular_max_, v_angular_min_, kp, kd, ki);

        this->exp_yaw_ = -M_PI / 2;

        this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_control_result", 10);

        this->yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>("odom2chassis_yaw", 10, std::bind(&SentryControlNode::handle_yaw_message, this, std::placeholders::_1));
        this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav2_result", 10, std::bind(&SentryControlNode::cmd_vel_callback, this, std::placeholders::_1));
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&SentryControlNode::timer_callback, this));
    }

    void SentryControlNode::load_params()
    {
        this->declare_parameter<int32_t>("dt", 50);
        this->declare_parameter<double>("pid.max_", 3.0);
        this->declare_parameter<double>("pid.min_", -3.0);
        this->declare_parameter<double>("pid.kp", 3.0);
        this->declare_parameter<double>("pid.ki", 0.1);
        this->declare_parameter<double>("pid.kd", 0.3);


        this->get_parameter("dt", dt_);
        this->get_parameter("pid.max_", v_angular_max_);
        this->get_parameter("pid.min_", v_angular_min_);
        this->get_parameter("pid.kp", kp);
        this->get_parameter("pid.ki", ki);
        this->get_parameter("pid.kd", kd);

    }

    void SentryControlNode::timer_callback()
    {
        if(!nav_start)
            return;
        this->exp_spin_ =  this->pid_->calculate(this->exp_yaw_, this->yaw_);
        RCLCPP_INFO(this->get_logger(), "yaw_: %.2lf exp_yaw_: %.2lf exp_spin_speed: %.2lf", this->yaw_, this->exp_yaw_, this->exp_spin_);
        this->out_cmd_vel_.angular.set__z(exp_spin_);
        this->cmd_vel_pub_->publish(this->out_cmd_vel_);
    }

    void SentryControlNode::handle_yaw_message(const std_msgs::msg::Float32::SharedPtr msg)
    {
        this->yaw_ = msg->data;
    }

    void SentryControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist msg)
    {
        this->out_cmd_vel_ = msg;
        nav_start = true;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sentry_control::SentryControlNode);