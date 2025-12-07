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
        // RCLCPP_INFO(this->get_logger(), "Parameters loaded: dt=%d, max=%f, min=%f, kp=%f, kd=%f, ki=%f, deadband=%f", dt_, v_angular_max_, v_angular_min_, kp, kd, ki, deadband);

        pid_ = std::make_shared<PID>(dt_/1000.0, v_angular_max_, v_angular_min_, kp, kd, ki, deadband);


        this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_control_result", 10);

        this->yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>("odom2chassis_yaw", 10, std::bind(&SentryControlNode::yaw_callback, this, std::placeholders::_1));
        this->chassis_mod_sub_ = this->create_subscription<control_interface::msg::ChassisMod>("chassis_mod", 10, std::bind(&SentryControlNode::chassis_mod_callback, this, std::placeholders::_1));
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
        this->declare_parameter<double>("pid.deadband", 0.05);


        this->get_parameter("dt", dt_);
        this->get_parameter("pid.max_", v_angular_max_);
        this->get_parameter("pid.min_", v_angular_min_);
        this->get_parameter("pid.kp", kp);
        this->get_parameter("pid.ki", ki);
        this->get_parameter("pid.kd", kd);
        this->get_parameter("pid.deadband", deadband);
    }

    void SentryControlNode::timer_callback()
    {
        if(!nav_start)
            return;
        if(out_cmd_vel_ == nullptr)
        {
            RCLCPP_WARN(this->get_logger(), "cmd_vel input is empty!!!");
            return;
        }
        if(this->chassis_mod_ != nullptr)
        {
            switch (chassis_mod_->type)
            {
            case control_interface::msg::ChassisMod::AIMANGLE:
                this->exp_spin_ =  this->pid_->calculate(this->yaw_, this->chassis_mod_->aim_angle);
                if(std::abs(this->yaw_ - this->chassis_mod_->aim_angle) > deadband)
                {
                    this->out_cmd_vel_->linear.set__x(0.0);
                    this->out_cmd_vel_->linear.set__y(0.0);
                }
                break;
            case control_interface::msg::ChassisMod::AIMSPEED:
                this->exp_spin_ = chassis_mod_->aim_speed;
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "chassis_mod_ type is not valid");
                return;
            }
        }else
        {
            RCLCPP_WARN(this->get_logger(), "chassis_mod_ is nullptr");
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "yaw_: %.2lf exp_yaw_: %.2lf exp_spin_speed: %.2lf", this->yaw_, this->exp_yaw_, this->exp_spin_);
        this->out_cmd_vel_->angular.set__z(exp_spin_);
        this->cmd_vel_pub_->publish(*this->out_cmd_vel_);
    }



    void SentryControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist msg)
    {
        this->out_cmd_vel_ = std::make_shared<geometry_msgs::msg::Twist>(msg);
        nav_start = true;
    }

    void SentryControlNode::chassis_mod_callback(const control_interface::msg::ChassisMod msg)
    {
        this->chassis_mod_ = std::make_shared<control_interface::msg::ChassisMod>(msg);
    }

    void SentryControlNode::yaw_callback(const std_msgs::msg::Float32 msg)
    {
        this->yaw_ = msg.data;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sentry_control::SentryControlNode);