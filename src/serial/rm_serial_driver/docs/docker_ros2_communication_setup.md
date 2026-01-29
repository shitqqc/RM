# Docker 容器间 ROS2 通信配置指南

## 前提条件

两个容器已经创建：
- `self_nav` 容器
- `rm_vision` 容器

## 配置方法

### 方法1：修改现有容器配置（推荐）

#### 步骤1：停止容器

```bash
docker stop self_nav rm_vision
```

#### 步骤2：删除现有容器（保留镜像）

```bash
docker rm self_nav rm_vision
```

#### 步骤3：重新创建容器（添加通信配置）

**self_nav 容器：**

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \          # 串口设备
  --network host \                  # 使用主机网络（关键！）
  -e ROS_DOMAIN_ID=0 \             # ROS2域名ID（关键！）
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \  # DDS实现（可选）
  self_nav:latest
```

**rm_vision 容器：**

```bash
docker run -it --name rm_vision \
  --network host \                  # 使用主机网络（关键！）
  -e ROS_DOMAIN_ID=0 \             # 相同的ROS2域名ID（关键！）
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \  # DDS实现（可选）
  rm_vision:latest
```

**关键配置说明**：
- `--network host`：使用主机网络模式，容器直接使用主机网络，便于ROS2通信
- `-e ROS_DOMAIN_ID=0`：设置ROS2域名ID，**两个容器必须相同**
- `-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp`：指定DDS实现（可选，默认即可）

---

### 方法2：修改运行中的容器（临时方案）

如果容器正在运行，可以临时设置环境变量：

#### 在 self_nav 容器中：

```bash
docker exec -it self_nav bash

# 设置环境变量
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 验证
echo $ROS_DOMAIN_ID
```

#### 在 rm_vision 容器中：

```bash
docker exec -it rm_vision bash

# 设置环境变量
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 验证
echo $ROS_DOMAIN_ID
```

**注意**：这种方法只在当前会话有效，容器重启后需要重新设置。

---

### 方法3：使用 Docker Compose（推荐用于生产环境）

创建 `docker-compose.yml` 文件：

```yaml
version: '3.8'

services:
  self_nav:
    container_name: self_nav
    image: self_nav:latest
    network_mode: host
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    volumes:
      - /home/rm/self_nav:/workspace
    stdin_open: true
    tty: true

  rm_vision:
    container_name: rm_vision
    image: rm_vision:latest
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    volumes:
      - /home/rm/rm_vision:/workspace
    stdin_open: true
    tty: true
```

启动：

```bash
docker-compose up -d
```

---

## 验证通信配置

### 步骤1：在两个容器中启动ROS2节点

**self_nav 容器：**

```bash
docker exec -it self_nav bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动串口桥接节点
ros2 launch rm_serial_driver serial_bridge.launch.py
```

**rm_vision 容器（新终端）：**

```bash
docker exec -it rm_vision bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动串口驱动节点
ros2 launch rm_serial_driver serial_driver.launch.py
```

### 步骤2：检查话题连接

在任一容器中：

```bash
# 查看所有话题
ros2 topic list

# 应该能在两个容器中看到相同的话题：
# /armor_solver/cmd_gimbal
# /serial/vision_packet
# /serial/gimbal_joint_state
# /task_mode
# /record_controller
```

### 步骤3：检查话题信息

```bash
# 检查话题的发布者和订阅者
ros2 topic info /serial/vision_packet

# 应该显示：
# Publisher count: 1  (self_nav容器中的串口桥接节点)
# Subscription count: 1  (rm_vision容器中的串口驱动节点)
```

### 步骤4：测试数据流

```bash
# 在self_nav容器中发布测试数据
ros2 topic pub /test_topic std_msgs/msg/String "data: 'hello from self_nav'"

# 在rm_vision容器中订阅
ros2 topic echo /test_topic
# 应该能看到消息
```

---

## 常见问题排查

### 问题1：两个容器看不到相同的话题

**原因**：ROS_DOMAIN_ID 不一致或网络配置问题

**解决方案**：
```bash
# 检查环境变量
docker exec self_nav env | grep ROS_DOMAIN_ID
docker exec rm_vision env | grep ROS_DOMAIN_ID
# 应该都显示 ROS_DOMAIN_ID=0

# 如果不同，重新创建容器或使用docker-compose
```

### 问题2：话题有发布者但没有订阅者

**原因**：节点启动顺序问题或话题名称不匹配

**解决方案**：
1. 确保串口桥接节点先启动
2. 检查话题名称是否完全一致（区分大小写）
3. 检查节点日志是否有错误

### 问题3：网络连接问题

**原因**：容器网络配置不正确

**解决方案**：
```bash
# 检查容器网络模式
docker inspect self_nav | grep NetworkMode
docker inspect rm_vision | grep NetworkMode
# 应该都显示 "host"

# 如果不同，重新创建容器时使用 --network host
```

### 问题4：DDS发现失败

**原因**：防火墙或网络配置阻止了DDS通信

**解决方案**：
```bash
# 检查端口（DDS默认使用7400-7500端口）
netstat -tuln | grep -E "7400|7500"

# 如果使用防火墙，需要开放这些端口
# 或者使用 --network host 绕过防火墙
```

---

## 高级配置（可选）

### 使用自定义DDS配置

如果需要更精细的控制，可以配置DDS：

**创建DDS配置文件** `fastdds_profile.xml`：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <use_SIMPLE_EndpointDiscoveryProtocol>true</use_SIMPLE_EndpointDiscoveryProtocol>
                    <use_STATIC_EndpointDiscoveryProtocol>false</use_STATIC_EndpointDiscoveryProtocol>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

**在容器中使用**：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_profile.xml
```

---

## 快速检查清单

- [ ] 两个容器都使用 `--network host`
- [ ] 两个容器都设置 `ROS_DOMAIN_ID=0`（相同值）
- [ ] 串口桥接节点已启动（self_nav容器）
- [ ] 串口驱动节点已启动（rm_vision容器）
- [ ] 使用 `ros2 topic list` 能看到相同的话题
- [ ] 使用 `ros2 topic info` 能看到发布者和订阅者

---

## 总结

**最简单的配置方法**：

1. 重新创建容器时添加：
   - `--network host`
   - `-e ROS_DOMAIN_ID=0`

2. 两个容器使用相同的 `ROS_DOMAIN_ID`

3. 使用 `--network host` 模式（最简单，推荐）

这样就可以实现两个容器之间的ROS2话题通信了！



