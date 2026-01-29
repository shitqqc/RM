# 视觉模块消息集成实现指南

## 当前状态

✅ **已完成**：
- 添加了 `VisionSendPacket` 数据包结构（rm_vision格式）
- 更新了launch文件，订阅话题改为 `armor_solver/cmd_gimbal`
- 创建了占位符实现

⚠️ **待完成**：
- 添加 `auto_aim_interfaces` 依赖
- 实现完整的 `GimbalCmd` 消息处理

## 实现步骤

### 步骤1：添加 auto_aim_interfaces 依赖

#### 方法A：从 rm_vision 工作空间复制（推荐）

```bash
cd /home/rm/self_nav/src
cp -r /home/rm/rm_vision/src/auto_aim_interfaces .
```

#### 方法B：从GitHub克隆

```bash
cd /home/rm/self_nav/src
git clone https://github.com/chenjunnn/rm_auto_aim.git
# auto_aim_interfaces 在 rm_auto_aim 目录中
```

### 步骤2：更新 package.xml

在 `package.xml` 中添加依赖：

```xml
<depend>auto_aim_interfaces</depend>
```

### 步骤3：修改 serial_bridge_node.hpp

在头文件中修改订阅类型：

```cpp
// 修改前：
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gimbal_cmd_sub_;

// 修改后：
rclcpp::Subscription<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
```

并修改函数签名：

```cpp
// 修改前：
void sendVisionData(const std_msgs::msg::Float32::SharedPtr msg);

// 修改后：
void sendVisionData(const auto_aim_interfaces::msg::GimbalCmd::ConstSharedPtr msg);
```

### 步骤4：修改 serial_bridge_node.cpp

#### 4.1 添加头文件

```cpp
#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
```

#### 4.2 修改订阅代码

```cpp
// 修改前：
gimbal_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
  "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
  std::bind(&SerialBridgeNode::sendVisionData, this, std::placeholders::_1));

// 修改后：
gimbal_cmd_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCmd>(
  "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
  std::bind(&SerialBridgeNode::sendVisionData, this, std::placeholders::_1));
```

#### 4.3 实现 sendVisionData 函数

```cpp
void SerialBridgeNode::sendVisionData(
  const auto_aim_interfaces::msg::GimbalCmd::ConstSharedPtr msg)
{
  try {
    VisionSendPacket packet;
    packet.header = 0xA5;
    packet.yaw = static_cast<float>(msg->yaw);
    packet.pitch = static_cast<float>(msg->pitch);
    packet.yaw_diff = static_cast<float>(msg->yaw_diff);
    packet.pitch_diff = static_cast<float>(msg->pitch_diff);
    packet.distance = static_cast<float>(msg->distance);
    packet.fire = msg->fire_advice ? 1 : 0;
    
    // 时间戳处理（可选）
    // 可以从 msg->header.stamp 计算 cap_timestamp
    packet.cap_timestamp = 0;  // 根据实际需求设置
    packet.t_offset = 0;       // 根据实际需求设置
    
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);
    RCLCPP_DEBUG(
      get_logger(),
      "Send vision packet: yaw=%.2f, pitch=%.2f, yaw_diff=%.2f, pitch_diff=%.2f, "
      "distance=%.2f, fire=%d",
      packet.yaw, packet.pitch, packet.yaw_diff, packet.pitch_diff,
      packet.distance, packet.fire);
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending vision data: %s", ex.what());
    reopenPort();
  }
}
```

### 步骤5：编译

```bash
cd /home/rm/self_nav
colcon build --packages-select auto_aim_interfaces rm_serial_driver
source install/setup.bash
```

## 验证

启动串口桥接节点后，检查话题：

```bash
# 查看话题列表
ros2 topic list | grep armor_solver

# 查看话题信息
ros2 topic info /armor_solver/cmd_gimbal

# 查看话题数据（如果视觉模块正在运行）
ros2 topic echo /armor_solver/cmd_gimbal
```

## 参考文件

- 完整实现示例：`src/serial_bridge_node_gimbal_cmd.cpp`
- 详细说明：`docs/vision_message_integration.md`



