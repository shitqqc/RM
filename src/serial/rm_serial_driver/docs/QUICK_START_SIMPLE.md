# 快速启动指南（简化版）

## 启动步骤

### 1. 启动 self_nav 容器（需要串口设备）

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  self_nav:latest
```

### 2. 在 self_nav 容器中启动串口桥接节点

```bash
# 进入容器后
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_serial_driver serial_bridge.launch.py
```

### 3. 启动 rm_vision 容器（不需要串口设备）

```bash
docker run -it --name rm_vision \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  rm_vision:latest
```

### 4. 在 rm_vision 容器中启动串口驱动节点

```bash
# 进入容器后
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

## 验证

在任一容器中：

```bash
# 查看话题
ros2 topic list

# 检查视觉数据包
ros2 topic echo /serial/vision_packet

# 检查云台状态
ros2 topic echo /serial/gimbal_joint_state
```

## 关键点

1. ✅ **self_nav 容器**：需要 `--device=/dev/ttyACM0`，运行串口桥接节点
2. ✅ **rm_vision 容器**：不需要串口设备，运行修改后的串口驱动节点
3. ✅ **两个容器**：都使用 `--network host` 和相同的 `ROS_DOMAIN_ID=0`
4. ✅ **启动顺序**：先启动串口桥接节点，再启动 rm_vision 节点



