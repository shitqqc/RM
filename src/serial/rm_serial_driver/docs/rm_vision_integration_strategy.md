# rm_vision 串口驱动集成策略

## 问题

现在有了串口桥接节点统一管理串口，`rm_vision` 容器中的 `rm_serial_driver` 是否还需要？

## 答案：需要，但需要修改

### 为什么需要保留？

`rm_vision` 中的 `rm_serial_driver` 有重要的业务逻辑：

1. **参数管理**：根据 `detect_color` 设置 `armor_detector` 参数
2. **服务调用**：调用 `/tracker/reset` 和 `/tracker/change` 服务
3. **任务模式判断**：根据 `game_time` 判断任务模式（aim/small_buff/large_buff/auto）
4. **话题发布**：
   - `/task_mode` - 任务模式
   - `/record_controller` - 录制控制
   - `/aiming_point` - 可视化Marker
   - `/latency` - 延迟信息
5. **TF发布**：发布 `odom -> gimbal_link` 变换

这些业务逻辑**不应该**放在串口桥接节点中，因为：
- 它们是视觉模块特定的逻辑
- 需要访问视觉模块的服务（tracker、detector）
- 保持模块职责分离

### 如何修改？

#### 方案：修改 rm_vision 中的 rm_serial_driver

**修改前**：
- 直接访问串口设备
- 接收串口数据并处理

**修改后**：
- **不再直接访问串口**
- 订阅串口桥接节点发布的视觉数据包话题
- 保留所有业务逻辑处理

#### 实现步骤

1. **在串口桥接节点中发布原始数据包**
   - 发布视觉数据包的原始字节流，或
   - 发布结构化的视觉数据包消息

2. **修改 rm_vision 中的 rm_serial_driver**
   - 移除串口访问代码
   - 订阅串口桥接节点发布的话题
   - 保留所有业务逻辑

## 数据包格式兼容性

注意：两个工作空间的数据包格式略有不同：

- **self_nav**: `VisionReceivePacket` (字段名：localColor, taskMode, aimX/aimY/aimZ, gameTime, rTimestamp)
- **rm_vision**: `ReceivePacket` (字段名：detect_color, task_mode, aim_x/aim_y/aim_z, game_time, timestamp)

字段名不同，但结构相同。需要在串口桥接节点中转换或统一格式。

## 推荐架构

```
┌─────────────────────┐                    ┌─────────────────────┐
│   self_nav          │                    │   rm_vision         │
│   Container         │                    │   Container         │
│                     │                    │                     │
│  ┌───────────────┐  │                    │  ┌───────────────┐  │
│  │ Serial Bridge │  │                    │  │ rm_serial_    │  │
│  │ Node           │  │                    │  │ driver        │  │
│  │                │  │                    │  │ (修改后)      │  │
│  │ - 管理串口     │  │                    │  │               │  │
│  │ - 接收数据     │  │                    │  │ - 订阅视觉    │  │
│  │ - 发布原始数据 │──┼──ROS2话题─────────>│  │   数据包话题  │  │
│  │                │  │                    │  │ - 处理业务    │  │
│  │                │  │                    │  │   逻辑        │  │
│  └───────┬───────┘  │                    │  └───────────────┘  │
│          │          │                    │                     │
│          │          │                    │                     │
└──────────┼──────────┘                    └─────────────────────┘
           │
           ▼
     /dev/ttyACM0
```

## 总结

- ✅ **保留** `rm_vision` 中的 `rm_serial_driver`
- ✅ **修改**为不直接访问串口，而是订阅话题
- ✅ **保留**所有业务逻辑处理
- ✅ **串口桥接节点**负责统一管理串口并发布数据



