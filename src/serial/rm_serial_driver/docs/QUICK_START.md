# 串口整合快速开始

## 问题

两个独立的 Docker 容器（`self_nav` 和 `rm_vision`）都需要访问同一个串口设备 `/dev/ttyACM0`，但串口设备不能同时被两个进程打开。

## 解决方案

使用**串口桥接节点**统一管理串口，通过 ROS2 话题在容器间转发数据。

## 快速开始

### 1. 编译代码

在 `self_nav` 工作空间中：

```bash
cd /path/to/self_nav
colcon build --packages-select rm_serial_driver
source install/setup.bash
```

### 2. 配置 Docker 容器

#### self_nav 容器（需要串口设备）

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  self_nav:latest
```

#### rm_vision 容器（不需要直接访问串口）

```bash
docker run -it --name rm_vision \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  rm_vision:latest
```

**关键点**：
- 两个容器都使用 `--network host` 以便ROS2通信
- 两个容器使用相同的 `ROS_DOMAIN_ID=0`
- 只有 `self_nav` 容器需要 `--device=/dev/ttyACM0`

### 3. 启动串口桥接节点

在 `self_nav` 容器中：

```bash
ros2 launch rm_serial_driver serial_bridge.launch.py
```

### 4. 验证

在任一容器中检查话题：

```bash
ros2 topic list
# 应该能看到：
# - /cmd_vel (导航模块发布)
# - /gimbal_topic (视觉模块发布)
# - /serial/gimbal_joint_state (串口桥接节点发布)
```

## 话题说明

### 输入话题（串口桥接节点订阅）

| 话题名称 | 类型 | 来源 | 说明 |
|---------|------|------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | self_nav容器 | 导航速度命令 |
| `/navi/status` | `std_msgs/msg/Int32` | self_nav容器 | 导航状态 |
| `/navi/sentry_cmd` | `std_msgs/msg/Int32` | self_nav容器 | 哨兵命令 |
| `/gimbal_topic` | `std_msgs/msg/Float32` | rm_vision容器 | 视觉云台命令 |

### 输出话题（串口桥接节点发布）

| 话题名称 | 类型 | 说明 |
|---------|------|------|
| `/serial/gimbal_joint_state` | `sensor_msgs/msg/JointState` | 云台关节状态（从串口接收） |
| `/serial/vision_state` | `std_msgs/msg/String` | 视觉状态（从串口接收） |

## 自定义配置

如果需要修改话题名称或串口参数，编辑：

1. **串口参数**：`config/serial_driver.yaml`
2. **话题映射**：`launch/serial_bridge.launch.py` 中的 `remappings`

## 故障排查

### 容器间无法通信

```bash
# 检查ROS2域名ID
echo $ROS_DOMAIN_ID  # 两个容器应该都是 0

# 检查网络
ping <other_container_ip>
```

### 串口无法打开

```bash
# 检查设备权限
ls -l /dev/ttyACM0

# 添加权限
sudo chmod 666 /dev/ttyACM0

# 检查用户组
groups  # 应该在 dialout 组中
```

### 话题无法接收

```bash
# 检查话题是否存在
ros2 topic list

# 检查话题信息
ros2 topic info /cmd_vel

# 查看话题数据
ros2 topic echo /cmd_vel
```

## 下一步

- 查看详细文档：`docs/docker_integration_guide.md`
- 查看架构设计：`docs/serial_bridge_solution.md`



