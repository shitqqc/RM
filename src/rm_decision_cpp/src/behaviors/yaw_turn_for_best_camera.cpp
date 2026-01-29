#include "rm_decision_cpp/behaviors/yaw_turn_for_best_camera.hpp"

using namespace BT;

namespace rm_decision
{

YawTurnForBestCamera::YawTurnForBestCamera(
  const std::string & name,
  const NodeConfig & config,
  std::shared_ptr<rclcpp::Node> node)
: SyncActionNode(name, config), node_(std::move(node))
{
  // 通过参数允许配置 yaw 偏移话题名称，默认使用 "omni_aim/yaw_offset"
  std::string yaw_offset_topic = "omni_aim/yaw_offset";
  node_->get_parameter_or<std::string>("yaw_offset_topic", yaw_offset_topic, "omni_aim/yaw_offset");

  yaw_offset_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    yaw_offset_topic, rclcpp::QoS(10));

  RCLCPP_INFO(node_->get_logger(),
    "YawTurnForBestCamera initialized, publishing yaw_offset to topic: %s", yaw_offset_topic.c_str());
}

NodeStatus YawTurnForBestCamera::tick()
{
  // 从黑板获取 best_camera 信息
  auto best_cam_res = getInput<int>("best_camera");
  if (!best_cam_res) {
    RCLCPP_WARN(node_->get_logger(), "YawTurnForBestCamera: failed to read port [best_camera]");
    return NodeStatus::FAILURE;
  }
  int best_cam = best_cam_res.value();

  // -1 或 0：无目标或主相机优先，保持当前自瞄，不额外转动
  if (best_cam < 0 || best_cam == 0) {
    // 发布 0，表示“不需要转向”，串口侧保持视觉原始 target_yaw
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data = 0.0f;
    yaw_offset_pub_->publish(offset_msg);
    RCLCPP_DEBUG(node_->get_logger(),
      "YawTurnForBestCamera: best_camera=%d, publish yaw_offset=0 (no turn).", best_cam);
    return NodeStatus::SUCCESS;
  }

  // 1 = 左相机优先 -> 向左转 110 度
  // 2 = 右相机优先 -> 向右转 110 度
  const double yaw_deg = 110.0;
  const double yaw_rad = yaw_deg * M_PI / 180.0;
  double yaw_delta = 0.0;

  if (best_cam == 1) {
    yaw_delta = +yaw_rad;  // 左相机：左转
  } else if (best_cam == 2) {
    yaw_delta = -yaw_rad;  // 右相机：右转
  } else {
    // 未知编码，直接返回成功但不做动作，避免异常行为
    RCLCPP_WARN(node_->get_logger(),
      "YawTurnForBestCamera: unknown best_camera=%d, skip yaw turn.", best_cam);
    return NodeStatus::SUCCESS;
  }

  std_msgs::msg::Float32 offset_msg;
  offset_msg.data = static_cast<float>(yaw_delta);
  yaw_offset_pub_->publish(offset_msg);

  RCLCPP_INFO(node_->get_logger(),
    "YawTurnForBestCamera: best_camera=%d, publish yaw_offset=%.3f rad (%.1f deg)",
    best_cam, yaw_delta, yaw_deg * (yaw_delta > 0 ? 1.0 : -1.0));

  // 本节点只负责一次性下发转向命令，后续由自瞄继续工作，因此直接返回 SUCCESS
  return NodeStatus::SUCCESS;
}

PortsList YawTurnForBestCamera::providedPorts()
{
  // 从 Topics2Blackboard 获得 best_camera 即可
  const char * desc = "Best camera index from Topics2Blackboard (0=HIK,1=left,2=right,-1=none).";
  return { InputPort<int>("best_camera", desc) };
}

}  // namespace rm_decision


