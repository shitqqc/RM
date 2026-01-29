# Docker 容器间串口整合指南

## 概述

本指南说明如何在两个独立的 Docker 容器（`self_nav` 和 `rm_vision`）之间实现串口通信整合。

## 架构设计

```
┌─────────────────────┐                    ┌─────────────────────┐
│   self_nav          │                    │   rm_vision         │
│   Container         │                    │   Container         │
│                     │                    │                     │
│  ┌───────────────┐  │                    │  ┌───────────────┐  │
│  │ Navigation    │  │                    │  │ Vision        │  │
│  │ Module        │  │                    │  │ Module        │  │
│  └───────┬───────┘  │                    │  └───────┬───────┘  │
│          │          │                    │          │          │
│          │ cmd_vel  │                    │          │ gimbal_  │
│          │ navi/    │                    │          │ cmd      │
│          │ status   │                    │          │          │
└──────────┼──────────┘                    └──────────┼──────────┘
           │                                        │
           │         ROS2 Topics (跨容器通信)        │
           │                                        │
           └────────────────────────────────────┘
                           │
                           ▼
           ┌─────────────────────────────┐
           │  Serial Bridge Node         │
           │  (在 self_nav 容器中运行)    │
           │                             │
           │  - 订阅导航数据             │
           │  - 订阅视觉数据             │
           │  - 统一管理串口             │
           │  - 分发接收到的数据         │
           └──────────────┬──────────────┘
                          │
                          ▼
                   /dev/ttyACM0
                   (串口设备)
```

## 实施步骤

### 1. Docker 配置

#### self_nav 容器配置

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \          # 挂载串口设备
  --network host \                  # 使用主机网络（便于ROS2通信）
  -e ROS_DOMAIN_ID=0 \             # 设置ROS2域名ID
  -v /dev:/dev \                    # 挂载设备目录
  self_nav:latest
```

#### rm_vision 容器配置

```bash
docker run -it --name rm_vision \
  --network host \                  # 使用主机网络（便于ROS2通信）
  -e ROS_DOMAIN_ID=0 \             # 设置相同的ROS2域名ID
  rm_vision:latest
```

**重要**：两个容器必须使用相同的 `ROS_DOMAIN_ID`，才能通过ROS2话题通信。

### 2. 启动串口桥接节点

在 `self_nav` 容器中启动串口桥接节点：

```bash
ros2 launch rm_serial_driver serial_bridge.launch.py
```

### 3. 话题映射

#### 导航模块发布的话题（self_nav容器）
- `cmd_vel` → 速度命令
- `navi/status` → 导航状态
- `navi/sentry_cmd` → 哨兵命令

#### 视觉模块发布的话题（rm_vision容器）
- `gimbal_topic` → 云台控制命令（需要根据实际话题名称调整）

#### 串口桥接节点发布的话题
- `serial/gimbal_joint_state` → 云台关节状态（从串口接收）
- `serial/vision_state` → 视觉状态（从串口接收）

### 4. 验证通信

#### 检查话题连接

在任一容器中运行：

```bash
# 查看所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /cmd_vel
ros2 topic info /gimbal_topic
ros2 topic info /serial/gimbal_joint_state
```

#### 测试数据流

```bash
# 在 self_nav 容器中发布测试数据
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}"

# 在 rm_vision 容器中发布测试数据
ros2 topic pub /gimbal_topic std_msgs/msg/Float32 "{data: 0.5}"

# 查看串口桥接节点日志
ros2 topic echo /serial/gimbal_joint_state
```

## 故障排查

### 问题1：容器间无法通信

**解决方案**：
1. 确保两个容器都使用 `--network host`
2. 确保 `ROS_DOMAIN_ID` 相同
3. 检查防火墙设置

### 问题2：串口设备无法访问

**解决方案**：
1. 确保 `self_nav` 容器使用 `--device=/dev/ttyACM0`
2. 检查串口设备权限：`sudo chmod 666 /dev/ttyACM0`
3. 检查用户是否在 `dialout` 组中

### 问题3：话题名称不匹配

**解决方案**：
1. 修改 `serial_bridge.launch.py` 中的 `remappings`
2. 或者使用 `ros2 topic remap` 进行重映射

## 高级配置

### 使用 Docker Compose

创建 `docker-compose.yml`：

```yaml
version: '3.8'

services:
  self_nav:
    image: self_nav:latest
    container_name: self_nav
    network_mode: host
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - /dev:/dev

  rm_vision:
    image: rm_vision:latest
    container_name: rm_vision
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
```

启动：

```bash
docker-compose up -d
```

## 注意事项

1. **串口独占性**：串口设备只能被一个进程打开，因此串口桥接节点必须在单一容器中运行。

2. **ROS2 通信**：确保两个容器能够通过ROS2 DDS进行通信，可能需要配置 `ROS_DOMAIN_ID` 或 DDS 配置。

3. **数据包格式**：确保 `self_nav` 和 `rm_vision` 使用的数据包格式兼容，或者串口桥接节点能够处理两种格式。

4. **性能考虑**：串口桥接节点会增加一定的延迟，对于实时性要求高的场景需要优化。



