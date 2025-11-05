#pragma once

#include "chrono"

#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"

namespace gimbal_spin

{
    class GimbalSpinNode : public rclcpp::Node
    {
    private:
        std::shared_ptr<float> gimbal_angle_;

        std::shared_ptr<rmoss_interfaces::msg::GimbalCmd> msg;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_spin_pub;

        void gimbal_spin_timer_callback();
        void cleanup();

    public:
        explicit GimbalSpinNode(const rclcpp::NodeOptions & options);
        ~GimbalSpinNode();
    };
} // namespace gimbal_spin
