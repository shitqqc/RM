#ifndef RM_DECISION_YAW_TURN_FOR_BEST_CAMERA_HPP_
#define RM_DECISION_YAW_TURN_FOR_BEST_CAMERA_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace BT;

namespace rm_decision
{

// 行为树节点：根据 Topics2Blackboard 提供的 best_camera 结果
// 决定是否向左/右偏转云台 110 度，然后交给自瞄继续工作。
// best_camera: 0=主HIK, 1=左USB, 2=右USB, -1=无目标
class YawTurnForBestCamera : public SyncActionNode
{
public:
  YawTurnForBestCamera(
    const std::string & name,
    const NodeConfig & config,
    std::shared_ptr<rclcpp::Node> node);

  ~YawTurnForBestCamera() override = default;

  NodeStatus tick() override;

  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_offset_pub_;
};

}  // namespace rm_decision

#endif  // RM_DECISION_YAW_TURN_FOR_BEST_CAMERA_HPP_


