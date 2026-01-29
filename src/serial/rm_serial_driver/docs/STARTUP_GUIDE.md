# 两个容器串口启动指南

## 架构概述

```
┌─────────────────────┐              ┌─────────────────────┐
│   self_nav          │              │   rm_vision         │
│   Container         │              │   Container         │
│                     │              │                     │
│  ┌───────────────┐  │              │  ┌───────────────┐  │
│  │ Serial Bridge │  │              │  │ rm_serial_   │  │
│  │ Node          │  │              │  │ driver       │  │
│  │               │  │              │  │ (修改后)     │  │
│  │ - 管理串口    │  │              │  │              │  │
│  │ - 接收数据    │  │              │  │ - 订阅话题   │  │
│  │ - 发布话题    │──┼──ROS2话题───►│  │ - 处理业务   │  │
│  └───────┬───────┘  │              │  └───────────────┘  │
│          │          │              │                     │
└──────────┼──────────┘              └─────────────────────┘
           │
           ▼
     /dev/ttyACM0
```

## 启动步骤

### 步骤1：配置 Docker 容器

#### self_nav 容器（需要串口设备）

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \          # 挂载串口设备（必须）
  --network host \                  # 使用主机网络（ROS2通信）
  -e ROS_DOMAIN_ID=0 \             # ROS2域名ID
  -v /home/rm/self_nav:/workspace \ # 挂载工作空间（可选）
  self_nav:latest
```

**关键参数**：
- `--device=/dev/ttyACM0`：挂载串口设备（**必须**）
- `--network host`：使用主机网络，便于ROS2跨容器通信
- `-e ROS_DOMAIN_ID=0`：设置ROS2域名ID

#### rm_vision 容器（不需要直接访问串口）

```bash
docker run -it --name rm_vision \
  --network host \                  # 使用主机网络（ROS2通信）
  -e ROS_DOMAIN_ID=0 \             # 相同的ROS2域名ID（必须）
  -v /home/rm/rm_vision:/workspace \ # 挂载工作空间（可选）
  rm_vision:latest
```

**关键参数**：
- `--network host`：使用主机网络，便于ROS2跨容器通信
- `-e ROS_DOMAIN_ID=0`：**必须与self_nav容器相同**
- **不需要** `--device=/dev/ttyACM0`（不再直接访问串口）

### 步骤2：在 self_nav 容器中启动串口桥接节点

进入 `self_nav` 容器：

```bash
# 如果容器已运行
docker exec -it self_nav bash

# 或者启动新容器后直接进入
```

在容器内：

```bash
# 1. 进入工作空间
cd /workspace  # 或你的工作空间路径

# 2. Source ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 检查串口设备
ls -l /dev/ttyACM0
# 应该能看到设备文件

# 4. 启动串口桥接节点
ros2 launch rm_serial_driver serial_bridge.launch.py
```

**预期输出**：
```
[INFO] [serial_bridge]: Starting Serial Bridge Node!
[INFO] [serial_bridge]: Ready to receive two packet types:
[INFO] [serial_bridge]:   - 0x6A: NaviReceivePacket (navigation data)
[INFO] [serial_bridge]:   - 0x5A: VisionReceivePacket (vision data)
```

### 步骤3：在 rm_vision 容器中启动串口驱动节点

进入 `rm_vision` 容器：

```bash
# 如果容器已运行
docker exec -it rm_vision bash

# 或者启动新容器后直接进入
```

在容器内：

```bash
# 1. 进入工作空间
cd /workspace  # 或你的工作空间路径

# 2. Source ROS2环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 启动串口驱动节点（修改后，不再直接访问串口）
ros2 launch rm_serial_driver serial_driver.launch.py
```

**预期输出**：
```
[INFO] [rm_serial_driver]: Start RMSerialDriver (modified: no direct serial access)!
[INFO] [rm_serial_driver]: Subscribing to serial bridge node for vision packets...
[INFO] [rm_serial_driver]: Subscribed to serial/vision_packet from serial bridge node
```

### 步骤4：验证通信

在任一容器中检查话题：

```bash
# 查看所有话题
ros2 topic list

# 应该能看到以下话题：
# /armor_solver/cmd_gimbal          (视觉模块发布)
# /serial/gimbal_joint_state        (串口桥接节点发布)
# /serial/vision_packet             (串口桥接节点发布，供rm_vision使用)
# /task_mode                        (rm_vision发布)
# /record_controller                (rm_vision发布)
# /aiming_point                     (rm_vision发布)
```

检查话题数据：

```bash
# 检查视觉数据包话题（应该能看到数据）
ros2 topic echo /serial/vision_packet

# 检查云台关节状态
ros2 topic echo /serial/gimbal_joint_state

# 检查任务模式
ros2 topic echo /task_mode
```

## 配置文件

### self_nav 容器配置

配置文件：`src/serial/rm_serial_driver/config/serial_driver.yaml`

```yaml
/rm_serial_driver:
  ros__parameters:
    nav_device_name: /dev/ttyACM0  # 串口设备路径
    baud_rate: 115200              # 波特率
    flow_control: none             # 流控制
    parity: none                    # 校验位
    stop_bits: "1"                  # 停止位
```

### rm_vision 容器配置

配置文件：`src/rm_serial_driver/config/serial_driver.yaml`

```yaml
/rm_serial_driver:
  ros__parameters:
    # 注意：修改后的节点不再需要串口参数
    # 但保留配置文件不会报错
    device_name: /dev/ttyACM0
    baud_rate: 115200
    flow_control: none
    parity: none
    stop_bits: "1"
```

## 启动顺序

**推荐启动顺序**：

1. ✅ 启动 self_nav 容器
2. ✅ 启动 rm_vision 容器
3. ✅ 在 self_nav 容器中启动串口桥接节点
4. ✅ 在 rm_vision 容器中启动串口驱动节点

**注意**：串口桥接节点必须先启动，因为 rm_vision 的节点需要订阅它发布的话题。

## 故障排查

### 问题1：串口桥接节点无法打开串口

**错误信息**：
```
[ERROR] [serial_bridge]: Error creating serial port: /dev/ttyACM0 - Permission denied
```

**解决方案**：
```bash
# 在宿主机上检查设备权限
ls -l /dev/ttyACM0

# 如果权限不足，添加用户到dialout组
sudo usermod -a -G dialout $USER
# 需要重新登录或重启

# 或者在Docker容器中临时修改权限
sudo chmod 666 /dev/ttyACM0
```

### 问题2：两个容器无法通信

**检查项**：
1. 两个容器是否都使用 `--network host`
2. 两个容器是否使用相同的 `ROS_DOMAIN_ID`
3. 检查话题列表：
   ```bash
   ros2 topic list
   # 应该能在两个容器中看到相同的话题
   ```

### 问题3：rm_vision 节点收不到数据

**检查项**：
1. 串口桥接节点是否已启动
2. 检查话题连接：
   ```bash
   ros2 topic info /serial/vision_packet
   # 应该显示有订阅者（rm_vision节点）
   ```
3. 检查串口是否有数据：
   ```bash
   # 在self_nav容器中
   ros2 topic echo /serial/vision_packet
   # 应该能看到数据
   ```

### 问题4：串口设备路径不同

如果串口设备不是 `/dev/ttyACM0`，修改配置文件：

```yaml
# self_nav/config/serial_driver.yaml
nav_device_name: /dev/ttyUSB0  # 改为实际设备路径
```

## 完整启动脚本示例

### self_nav 容器启动脚本

```bash
#!/bin/bash
# start_self_nav.sh

docker run -it --name self_nav \
  --device=/dev/ttyACM0 \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -v /home/rm/self_nav:/workspace \
  self_nav:latest \
  bash -c "cd /workspace && \
           source /opt/ros/humble/setup.bash && \
           source install/setup.bash && \
           ros2 launch rm_serial_driver serial_bridge.launch.py"
```

### rm_vision 容器启动脚本

```bash
#!/bin/bash
# start_rm_vision.sh

docker run -it --name rm_vision \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -v /home/rm/rm_vision:/workspace \
  rm_vision:latest \
  bash -c "cd /workspace && \
           source /opt/ros/humble/setup.bash && \
           source install/setup.bash && \
           ros2 launch rm_serial_driver serial_driver.launch.py"
```

## 总结

1. **self_nav 容器**：需要串口设备，运行串口桥接节点
2. **rm_vision 容器**：不需要串口设备，运行修改后的串口驱动节点
3. **通信方式**：通过ROS2话题跨容器通信
4. **启动顺序**：先启动串口桥接节点，再启动rm_vision节点



