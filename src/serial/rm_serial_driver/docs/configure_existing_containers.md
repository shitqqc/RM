# 配置已创建容器的ROS2通信

## 方法1：修改运行中的容器（快速方案）

### 步骤1：在容器中设置环境变量

**self_nav 容器：**

```bash
# 进入容器
docker exec -it self_nav bash

# 设置环境变量（当前会话有效）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 验证
echo $ROS_DOMAIN_ID
# 应该输出: 0
```

**rm_vision 容器：**

```bash
# 进入容器
docker exec -it rm_vision bash

# 设置环境变量（当前会话有效）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 验证
echo $ROS_DOMAIN_ID
# 应该输出: 0
```

**注意**：这种方法只在当前bash会话有效，容器重启后需要重新设置。

### 步骤2：在启动脚本中设置（持久化）

如果容器有启动脚本，可以在脚本中添加：

```bash
#!/bin/bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 其他启动命令...
```

---

## 方法2：重新创建容器（推荐，永久生效）

### 步骤1：停止并删除现有容器

```bash
# 停止容器
docker stop self_nav rm_vision

# 删除容器（保留镜像和数据卷）
docker rm self_nav rm_vision
```

### 步骤2：重新创建容器（添加通信配置）

**self_nav 容器：**

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \          # 串口设备
  --network host \                  # 关键：使用主机网络
  -e ROS_DOMAIN_ID=0 \             # 关键：设置ROS2域名ID
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \  # DDS实现
  self_nav:latest
```

**rm_vision 容器：**

```bash
docker run -it --name rm_vision \
  --network host \                  # 关键：使用主机网络
  -e ROS_DOMAIN_ID=0 \             # 关键：相同的ROS2域名ID
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \  # DDS实现
  rm_vision:latest
```

**关键配置**：
- `--network host`：使用主机网络，容器直接使用主机网络接口
- `-e ROS_DOMAIN_ID=0`：设置ROS2域名ID，**两个容器必须相同**

---

## 方法3：使用Docker Compose（最佳实践）

### 步骤1：创建 docker-compose.yml

在宿主机上创建文件：

```yaml
version: '3.8'

services:
  self_nav:
    container_name: self_nav
    image: self_nav:latest
    network_mode: host                    # 使用主机网络
    devices:
      - /dev/ttyACM0:/dev/ttyACM0         # 串口设备
    environment:
      - ROS_DOMAIN_ID=0                   # ROS2域名ID
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp # DDS实现
    volumes:
      - /home/rm/self_nav:/workspace      # 挂载工作空间（可选）
    stdin_open: true
    tty: true
    restart: unless-stopped

  rm_vision:
    container_name: rm_vision
    image: rm_vision:latest
    network_mode: host                    # 使用主机网络
    environment:
      - ROS_DOMAIN_ID=0                   # 相同的ROS2域名ID
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp # DDS实现
    volumes:
      - /home/rm/rm_vision:/workspace    # 挂载工作空间（可选）
    stdin_open: true
    tty: true
    restart: unless-stopped
```

### 步骤2：停止并删除现有容器

```bash
docker stop self_nav rm_vision
docker rm self_nav rm_vision
```

### 步骤3：使用docker-compose启动

```bash
# 启动容器
docker-compose up -d

# 查看日志
docker-compose logs -f

# 进入容器
docker-compose exec self_nav bash
docker-compose exec rm_vision bash
```

---

## 验证通信配置

### 步骤1：检查环境变量

**在 self_nav 容器中：**

```bash
docker exec self_nav env | grep ROS_DOMAIN_ID
# 应该输出: ROS_DOMAIN_ID=0
```

**在 rm_vision 容器中：**

```bash
docker exec rm_vision env | grep ROS_DOMAIN_ID
# 应该输出: ROS_DOMAIN_ID=0
```

### 步骤2：检查网络模式

```bash
docker inspect self_nav | grep NetworkMode
# 应该输出: "NetworkMode": "host"

docker inspect rm_vision | grep NetworkMode
# 应该输出: "NetworkMode": "host"
```

### 步骤3：测试ROS2通信

**在 self_nav 容器中：**

```bash
docker exec -it self_nav bash
source /opt/ros/humble/setup.bash

# 发布测试话题
ros2 topic pub /test_communication std_msgs/msg/String "data: 'hello from self_nav'" --once
```

**在 rm_vision 容器中（新终端）：**

```bash
docker exec -it rm_vision bash
source /opt/ros/humble/setup.bash

# 订阅测试话题
ros2 topic echo /test_communication
# 应该能看到消息: hello from self_nav
```

### 步骤4：检查话题列表

在任一容器中：

```bash
ros2 topic list

# 应该能在两个容器中看到相同的话题列表
```

---

## 如果容器已经在运行

### 临时方案：在运行的容器中设置

如果不想重启容器，可以在每个容器的bash会话中设置：

```bash
# self_nav容器
docker exec -it self_nav bash
export ROS_DOMAIN_ID=0
# 然后在这个bash会话中启动节点

# rm_vision容器
docker exec -it rm_vision bash
export ROS_DOMAIN_ID=0
# 然后在这个bash会话中启动节点
```

### 永久方案：修改容器配置

如果容器支持，可以修改容器的启动脚本或 `.bashrc`：

```bash
# 进入容器
docker exec -it self_nav bash

# 编辑 .bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc

# 重新加载
source ~/.bashrc
```

---

## 快速检查清单

- [ ] 两个容器都使用 `--network host` 或 `network_mode: host`
- [ ] 两个容器都设置了 `ROS_DOMAIN_ID=0`（相同值）
- [ ] 使用 `docker inspect` 验证网络模式
- [ ] 使用 `ros2 topic list` 验证话题可见性
- [ ] 使用测试话题验证数据流

---

## 常见问题

### Q: 容器已经运行，不想重启怎么办？

A: 使用**方法1**，在bash会话中设置环境变量。但注意每次新开bash会话都需要重新设置。

### Q: 如何让环境变量永久生效？

A: 使用**方法2**重新创建容器，或使用**方法3**的docker-compose。

### Q: 两个容器看不到相同的话题？

A: 检查：
1. `ROS_DOMAIN_ID` 是否相同
2. 网络模式是否为 `host`
3. 防火墙是否阻止了DDS通信（端口7400-7500）

### Q: 使用 `--network host` 安全吗？

A: 在开发环境中通常可以，但生产环境建议使用自定义网络并配置防火墙规则。

---

## 推荐方案

**对于已创建的容器**：
1. 如果容器正在运行且不想重启：使用方法1（临时设置）
2. 如果可以重启容器：使用方法2（重新创建）
3. 如果希望更好的管理：使用方法3（docker-compose）

**最简单的方法**：重新创建容器时添加 `--network host` 和 `-e ROS_DOMAIN_ID=0`



