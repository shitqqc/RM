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

        pid_ = std::make_shared<PID>(dt_/1000.0, v_angular_max_, v_angular_min_, kp, kd, ki, deadband);
        sentry_mod_ = std::make_shared<SentryMod>();
        path_checker_ = std::make_shared<path_checker>(have_bumpy_area_);

        path_checker_->set_bumpy_area(friend_bumpy_area_, enermy_bumpy_area_);

        this->chassis_mod_ = std::make_shared<control_interface::msg::ChassisMod>();

        this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_control_result", 10);

        this->yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>("odom2chassis_yaw", 10, std::bind(&SentryControlNode::yaw_callback, this, std::placeholders::_1));
        this->use_spin_sub_ = this->create_subscription<std_msgs::msg::Bool>("use_spin", 10, std::bind(&SentryControlNode::use_spin_callback, this, std::placeholders::_1));
        this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav2_result", 10, std::bind(&SentryControlNode::cmd_vel_callback, this, std::placeholders::_1));
        this->path_sub_ = this->create_subscription<nav_msgs::msg::Path>(path_topic_, 10, std::bind(&SentryControlNode::path_callback, this, std::placeholders::_1));
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&SentryControlNode::timer_callback, this));
    }

    void SentryControlNode::load_params()
    {
        this->declare_parameter<int32_t>("dt", 50);
        this->declare_parameter<bool>("have_bumpy_area", false);
        this->declare_parameter<std::string>("path_topic", "path");
        this->declare_parameter<double>("speed_limit", 1.0);
        this->declare_parameter<double>("deadband", 0.05);
        this->declare_parameter<double>("pid.max_", 3.0);
        this->declare_parameter<double>("pid.min_", -3.0);
        this->declare_parameter<double>("pid.kp", 3.0);
        this->declare_parameter<double>("pid.ki", 0.1);
        this->declare_parameter<double>("pid.kd", 0.3);
        this->declare_parameter<double>("friend_bumpy_area.x_max", 5.0);
        this->declare_parameter<double>("friend_bumpy_area.x_min", -5.0);
        this->declare_parameter<double>("friend_bumpy_area.y_max", 5.0);
        this->declare_parameter<double>("friend_bumpy_area.y_min", -5.0);
        this->declare_parameter<double>("enermy_bumpy_area.x_max", 5.0);
        this->declare_parameter<double>("enermy_bumpy_area.x_min", -5.0);
        this->declare_parameter<double>("enermy_bumpy_area.y_max", 5.0);
        this->declare_parameter<double>("enermy_bumpy_area.y_min", -5.0);

        this->get_parameter("dt", dt_);
        this->get_parameter("have_bumpy_area", have_bumpy_area_);
        this->get_parameter("path_topic", path_topic_);
        this->get_parameter("speed_limit", speed_limit_);
        this->get_parameter("deadband", deadband); 
        this->get_parameter("pid.max_", v_angular_max_);
        this->get_parameter("pid.min_", v_angular_min_);
        this->get_parameter("pid.kp", kp);
        this->get_parameter("pid.ki", ki);
        this->get_parameter("pid.kd", kd);
        this->get_parameter("friend_bumpy_area.x_max", friend_bumpy_area_.x_max);
        this->get_parameter("friend_bumpy_area.x_min", friend_bumpy_area_.x_min);
        this->get_parameter("friend_bumpy_area.y_max", friend_bumpy_area_.y_max);
        this->get_parameter("friend_bumpy_area.y_min", friend_bumpy_area_.y_min);
        this->get_parameter("enermy_bumpy_area.x_max", enermy_bumpy_area_.x_max);
        this->get_parameter("enermy_bumpy_area.x_min", enermy_bumpy_area_.x_min);
        this->get_parameter("enermy_bumpy_area.y_max", enermy_bumpy_area_.y_max);
        this->get_parameter("enermy_bumpy_area.y_min", enermy_bumpy_area_.y_min);
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
        sentry_mod_->getChassisMod(chassis_mod_);
        if (this->chassis_mod_ != nullptr)
        {
            // RCLCPP_INFO(this->get_logger(), "chassis_mod_ type: %d", chassis_mod_->type);
            switch (chassis_mod_->type)
            {
            case control_interface::msg::ChassisMod::AIMANGLE:
                this->chassis_mod_->aim_angle = this->get_aim_yaw();
                this->exp_spin_ =  this->pid_->calculate(this->yaw_, this->chassis_mod_->aim_angle);
                this->speed_limit(this->out_cmd_vel_);
                if(std::abs(exp_spin_) >  deadband * this->kp)
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
        this->out_cmd_vel_->angular.set__z(exp_spin_);
        this->cmd_vel_pub_->publish(*this->out_cmd_vel_);
    }

    double SentryControlNode::get_aim_yaw()
    {
        double aim_yaw_list_[4] = {0.0, -M_PI/2, M_PI, M_PI/2};
        auto current_yaw_ = this->yaw_;
        for(int i = 0; i < 4; i++)
        {
            if(std::abs(std::abs(current_yaw_) - aim_yaw_list_[i]) <= M_PI/4)
                return aim_yaw_list_[i];
        }
        return 0.0; // 默认返回值，如果没有匹配的角度
    }

    void SentryControlNode::speed_limit(geometry_msgs::msg::Twist::SharedPtr &cmd_vel)
    {
        double linear_speed = std::hypot(cmd_vel->linear.x, cmd_vel->linear.y);
        if(linear_speed > speed_limit_)
        {
            double scale = speed_limit_ / linear_speed;
            cmd_vel->linear.x *= scale;
            cmd_vel->linear.y *= scale;
        }
    }

    void SentryControlNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->out_cmd_vel_ = msg;
        nav_start = true;
    }

    void SentryControlNode::yaw_callback(const std_msgs::msg::Float32 msg)
    {
        this->yaw_ = msg.data;
    }

    void SentryControlNode::use_spin_callback(const std_msgs::msg::Bool msg)
    {
        this->sentry_mod_->useSpin(msg.data);
    }

    void SentryControlNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        this->path_checker_->get_path(msg);
        this->sentry_mod_->inBumpyArea(this->path_checker_->is_in_bumpy_area());
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sentry_control::SentryControlNode);