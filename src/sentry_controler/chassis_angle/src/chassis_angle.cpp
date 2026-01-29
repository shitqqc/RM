#include "chassis_angle/chassis_angle.hpp"

namespace chassis_angle
{
    ChassisAngleNode::ChassisAngleNode(const rclcpp::NodeOptions &options) : Node("chassis_angle", options)
    {
        this->load_param();

        RCLCPP_INFO(this->get_logger(), "chassis_frame: %s", this->chassis_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "odom_frame: %s", this->odom_frame.c_str());

        this->tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

        this->timer_ = this->create_wall_timer(std::chrono::microseconds(50), std::bind(&ChassisAngleNode::tf2_timer_callback, this));
        this->yaw_pub = this->create_publisher<std_msgs::msg::Float32>("odom2chassis_yaw", 10);
        
    }

    void ChassisAngleNode::load_param()
    {
        this->declare_parameter<std::string>("chassis_frame", "");
        this->declare_parameter<std::string>("odom_frame", "");

        this->get_parameter("chassis_frame", chassis_frame);
        this->get_parameter("odom_frame", odom_frame);
    }

    void ChassisAngleNode::tf2_timer_callback()
    {
        try
        {
            // 使用 tf2::TimePointZero 获取最新的可用变换，而不是使用当前时间
            // 这样可以避免时间外推错误，特别是在使用 sim_time 时
            auto trans = this->tf2_buffer_->lookupTransform(
                this->chassis_frame, 
                this->odom_frame, 
                tf2::TimePointZero);
            yaw_ = tf2::getYaw(trans.transform.rotation);

            //RCLCPP_INFO_STREAM(this->get_logger(), "yaw from" << odom_frame << " to " << chassis_frame << " : " << yaw_);

            msg.data = yaw_;

            yaw_pub->publish(msg);
        }
        catch(const std::exception& e)
        {
            // 使用节流警告，每5秒最多打印一次，避免在TF树建立前产生大量警告
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), 
                *this->get_clock(), 
                5000, 
                "Failed to get transform from '%s' to '%s': %s", 
                this->odom_frame.c_str(), 
                this->chassis_frame.c_str(), 
                e.what());
        }
    }

    void ChassisAngleNode::cleanup()
    {
        if(timer_)
        {
            timer_->cancel();
            timer_->reset();
        }

        tf2_buffer_.reset();
        tf2_listener_.reset();

        yaw_pub.reset();
    }

    ChassisAngleNode::~ChassisAngleNode()
    {
        this->cleanup();
    }

} // namespace chassis_angle

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(chassis_angle::ChassisAngleNode);
