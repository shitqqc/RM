# 视觉模块消息集成说明

## 问题

视觉模块（rm_vision）发送的消息类型是 `auto_aim_interfaces::msg::GimbalCmd`，话题是 `armor_solver/cmd_gimbal`。

但是 `self_nav` 工作空间可能没有 `auto_aim_interfaces` 包。

## 解决方案

### 方案1：添加 auto_aim_interfaces 依赖（推荐）

在 `self_nav` 工作空间中添加 `auto_aim_interfaces` 包：

```bash
cd /home/rm/self_nav/src
git clone <auto_aim_interfaces_repo> auto_aim_interfaces
# 或者从rm_vision工作空间复制
cp -r /home/rm/rm_vision/src/auto_aim_interfaces /home/rm/self_nav/src/
```

然后在 `package.xml` 中添加依赖：

```xml
<depend>auto_aim_interfaces</depend>
```

### 方案2：使用自定义消息结构

如果不想添加依赖，可以在串口桥接节点中创建兼容的消息结构。

## 当前实现状态

当前串口桥接节点：
- ✅ 已添加 `VisionSendPacket` 结构（rm_vision格式的数据包）
- ⚠️ 暂时使用 `std_msgs::msg::Float32` 作为占位符
- ⚠️ 需要实现完整的 `GimbalCmd` 消息处理

## 需要修改的地方

1. **修改订阅话题**：从 `gimbal_topic` 改为 `armor_solver/cmd_gimbal`
2. **修改消息类型**：从 `std_msgs::msg::Float32` 改为 `auto_aim_interfaces::msg::GimbalCmd`
3. **实现数据转换**：将 `GimbalCmd` 转换为 `VisionSendPacket`

## GimbalCmd 消息结构

```cpp
// auto_aim_interfaces::msg::GimbalCmd
std_msgs/Header header
float64 pitch
float64 yaw
float64 yaw_diff
float64 pitch_diff
float64 distance
bool fire_advice
```

## VisionSendPacket 数据包结构

```cpp
struct VisionSendPacket {
  uint8_t header = 0xA5;
  float yaw;
  float pitch;
  float yaw_diff;
  float pitch_diff;
  float distance;
  uint8_t fire;
  uint32_t cap_timestamp;
  uint16_t t_offset;
  uint16_t checksum;
}
```

## 转换逻辑

```cpp
VisionSendPacket packet;
packet.header = 0xA5;
packet.yaw = gimbal_cmd->yaw;
packet.pitch = gimbal_cmd->pitch;
packet.yaw_diff = gimbal_cmd->yaw_diff;
packet.pitch_diff = gimbal_cmd->pitch_diff;
packet.distance = gimbal_cmd->distance;
packet.fire = gimbal_cmd->fire_advice ? 1 : 0;
packet.cap_timestamp = 0;  // 根据实际需求设置
packet.t_offset = 0;       // 根据实际需求设置
```



