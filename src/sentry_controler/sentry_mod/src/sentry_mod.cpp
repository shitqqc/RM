#include "sentry_mod/sentry_mod.hpp"

namespace sentry_mod
{
    SentryMod::SentryMod(const rclcpp::NodeOptions & options) : Node("sentry_mod", options)
    {
        RCLCPP_INFO(this->get_logger(), "Sentry mod node started");

        this->load_params();

        chassis_mod_msg_ = std::make_shared<control_interface::msg::ChassisMod>();

        in_bumpy_area_ = false;
        use_spin_ = true;

        in_bumpy_area_sub_ = this->create_subscription<std_msgs::msg::Bool>("in_bumpy_area", 10, std::bind(&SentryMod::in_bumpy_area_callback, this, std::placeholders::_1));
        chassis_mod_pub_ = this->create_publisher<control_interface::msg::ChassisMod>("chassis_mod", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SentryMod::timer_callback, this));
    }

    void SentryMod::load_params()
    {
        this->declare_parameter<double>("spin_speed", 1.0);
        
        this->get_parameter("spin_speed", default_spin_speed_);
    }

    void SentryMod::timer_callback()
    {
        if(in_bumpy_area_)
        {
            this->chassis_mod_msg_->type = control_interface::msg::ChassisMod::AIMANGLE;
            this->chassis_mod_msg_->aim_angle = 0.0f;
            // RCLCPP_INFO(this->get_logger(), "in bumpy area, use aim angle 0.0f");
        }else if (!in_bumpy_area_ && use_spin_)
        {
            this->chassis_mod_msg_->type = control_interface::msg::ChassisMod::AIMSPEED;
            this->chassis_mod_msg_->aim_speed = default_spin_speed_;
            // RCLCPP_INFO(this->get_logger(), "not in bumpy area, use aim speed %f", default_spin_speed_);
        }else
        {
            this->chassis_mod_msg_->type = control_interface::msg::ChassisMod::AIMSPEED;
            this->chassis_mod_msg_->aim_speed = 0.0f;
        }
        chassis_mod_pub_->publish(*chassis_mod_msg_);
    }
    void SentryMod::in_bumpy_area_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        in_bumpy_area_ = msg->data;
    }
} // namespace sentry_mod

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sentry_mod::SentryMod)
