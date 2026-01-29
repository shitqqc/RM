# 串口整合方案：Docker 容器间串口通信

## 问题分析

当前情况：
- `self_nav` Docker 容器：需要发送导航数据（cmd_vel）和接收云台姿态
- `rm_vision` Docker 容器：需要发送视觉数据（gimbal_cmd）和接收下位机状态
- 两个容器都需要访问同一个串口设备 `/dev/ttyACM0`
- **问题**：串口设备不能同时被两个进程打开

## 解决方案

### 方案1：串口桥接节点（推荐）

创建一个统一的串口桥接节点，在一个容器中运行，通过 ROS2 话题与其他容器通信。

#### 架构设计

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  self_nav       │         │  Serial Bridge   │         │  rm_vision      │
│  Container      │         │  (任一容器)      │         │  Container      │
│                 │         │                  │         │                 │
│  cmd_vel ──────>│────────>│  nav_cmd_topic   │         │                 │
│  gimbal_topic   │         │                  │         │  gimbal_cmd ────>│
│                 │         │  vision_cmd_topic│<────────│                 │
│  gimbal_state <─│<────────│  gimbal_state    │         │                 │
│                 │         │                  │         │  vision_state <─│
│                 │         │  vision_state    │<────────│                 │
│                 │         │                  │         │                 │
│                 │         │  /dev/ttyACM0    │         │                 │
│                 │         │  (串口设备)      │         │                 │
└─────────────────┘         └──────────────────┘         └─────────────────┘
```

#### 实现步骤

1. **创建串口桥接节点**（在 self_nav 或 rm_vision 容器中运行）

2. **修改 Docker 配置**，确保串口设备可访问

3. **配置 ROS2 通信**，使两个容器能够通信

#### Docker 配置示例

**self_nav 容器启动：**
```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  self_nav:latest
```

**rm_vision 容器启动：**
```bash
docker run -it --name rm_vision \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  rm_vision:latest
```

**串口桥接节点（在 self_nav 容器中运行）：**
```bash
ros2 launch rm_serial_driver serial_bridge.launch.py
```

### 方案2：修改现有串口驱动支持多数据源

修改 `self_nav` 中的 `rm_serial_driver`，使其能够：
1. 接收来自导航模块的数据（cmd_vel）
2. 接收来自视觉模块的数据（通过 ROS2 话题）
3. 统一发送到串口
4. 接收串口数据并分发到不同话题

## 推荐实现：串口桥接节点

