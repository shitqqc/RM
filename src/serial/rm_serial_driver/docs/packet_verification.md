# 数据包接收验证

## 确认：串口桥接节点支持两种数据包

串口桥接节点 (`SerialBridgeNode`) **确实能够接收和处理两种数据包**：

### 1. 导航数据包 (NaviReceivePacket)

- **Header**: `0x6A`
- **结构**:
  ```cpp
  struct NaviReceivePacket {
    uint8_t header = 0x6A;
    float pitch;
    float yaw;
    uint16_t game_time;
    uint8_t game_progress;
    uint8_t navi_hp_state;
    uint16_t crcSum;
  }
  ```
- **处理逻辑**:
  - 接收数据包
  - 验证CRC校验
  - 发布到 `/serial/gimbal_joint_state` 话题（`sensor_msgs/msg/JointState`）
  - 包含云台的 pitch 和 yaw 角度

### 2. 视觉数据包 (VisionReceivePacket)

- **Header**: `0x5A`
- **结构**:
  ```cpp
  struct VisionReceivePacket {
    uint8_t header = 0x5A;
    uint8_t localColor : 1;
    uint8_t taskMode : 2;
    bool resetTracker : 1;
    uint8_t isplay : 1;
    bool targetChange : 1;
    uint8_t reserved : 2;
    float roll;
    float pitch;
    float yaw;
    float aimX;
    float aimY;
    float aimZ;
    uint16_t gameTime;
    uint32_t rTimestamp;
    uint16_t checkSum;
  }
  ```
- **处理逻辑**:
  - 接收数据包
  - 验证CRC校验
  - 发布到 `/serial/vision_state` 话题（`std_msgs/msg/String`）
  - 包含视觉系统的状态信息（姿态、瞄准点等）

## 代码位置

数据包接收处理在 `src/serial_bridge_node.cpp` 的 `receiveData()` 函数中：

```cpp
void SerialBridgeNode::receiveData()
{
  // ...
  if (header[0] == 0x6A) {
    // 处理导航数据包
    // ...
  } else if (header[0] == 0x5A) {
    // 处理视觉数据包
    // ...
  }
  // ...
}
```

## 验证方法

### 方法1：查看日志

启动串口桥接节点后，应该能看到初始化信息：

```
[INFO] Serial Bridge Node initialized successfully!
[INFO] Ready to receive two packet types:
[INFO]   - 0x6A: NaviReceivePacket (navigation data)
[INFO]   - 0x5A: VisionReceivePacket (vision data)
```

当接收到数据包时，会输出调试信息（如果日志级别设置为DEBUG）。

### 方法2：检查话题

```bash
# 查看发布的话题
ros2 topic list | grep serial

# 应该看到：
# /serial/gimbal_joint_state  (来自导航数据包)
# /serial/vision_state        (来自视觉数据包)

# 查看话题数据
ros2 topic echo /serial/gimbal_joint_state
ros2 topic echo /serial/vision_state
```

### 方法3：监控串口数据

```bash
# 使用串口监控工具（在另一个终端）
sudo cat /dev/ttyACM0 | hexdump -C

# 应该能看到：
# - 0x6A 开头的数据包（导航数据）
# - 0x5A 开头的数据包（视觉数据）
```

## 错误处理

如果数据包接收失败，会输出错误信息：

- **CRC错误**: `CRC error in Navi packet (0x6A)!` 或 `CRC error in Vision packet (0x5A)!`
- **无效Header**: `Invalid header: XX`（XX是无效的header值）
- **接收错误**: `Error while receiving data: ...`

## 注意事项

1. **数据包大小**: 确保接收的数据包大小正确
   - NaviReceivePacket: `sizeof(NaviReceivePacket) - 1`（减去header）
   - VisionReceivePacket: `sizeof(VisionReceivePacket) - 1`（减去header）

2. **CRC校验**: 所有数据包都经过CRC16校验，确保数据完整性

3. **线程安全**: 接收数据在独立线程中运行，不会阻塞主线程

4. **自动重连**: 如果串口连接失败，会自动尝试重新打开端口



