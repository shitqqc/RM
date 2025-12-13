#include "gimbal_spin/gimbal_spin.hpp"

namespace gimbal_spin
{
    GimbalSpinNode::GimbalSpinNode(const rclcpp::NodeOptions &options) : Node("gimbal_spin", options)
    {
        RCLCPP_INFO(this->get_logger(), "gimbal_spin_node start");

        gimbal_angle_ = std::make_shared<float>(0.0);
        msg = std::make_shared<rmoss_interfaces::msg::GimbalCmd>();

        this->gimbal_spin_pub = this->create_publisher<rmoss_interfaces::msg::GimbalCmd>("robot_base/gimbal_cmd", 10);

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GimbalSpinNode::gimbal_spin_timer_callback,this));
        
    }

    void GimbalSpinNode::gimbal_spin_timer_callback()
    {
        *gimbal_angle_ += 0.1;

        msg->yaw_type = rmoss_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
        msg->pitch_type = rmoss_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;

        msg->position.yaw = *gimbal_angle_;
        msg->position.pitch = 0.0;

        gimbal_spin_pub->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "gimbal angle: %.2lf", *gimbal_angle_);
    }

    void GimbalSpinNode::cleanup()
    {
        if(timer_)
        {
            timer_->cancel();
            timer_->reset();
        }

        msg.reset();
        gimbal_angle_.reset();
        gimbal_spin_pub.reset();
    }

    GimbalSpinNode::~GimbalSpinNode()
    {
        this->cleanup();
    }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gimbal_spin::GimbalSpinNode);
