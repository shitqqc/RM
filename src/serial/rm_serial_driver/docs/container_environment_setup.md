# 重新创建容器后的环境配置说明

## 简短回答

**不需要重新配置 ROS2 通信环境变量**（如果使用正确的方式创建容器），但**需要配置工作空间环境**。

## 详细说明

### 1. ROS2 通信环境变量（自动配置）

如果使用以下方式重新创建容器，**ROS2 通信环境变量会自动配置**：

```bash
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \
  --network host \
  -e ROS_DOMAIN_ID=0 \        # ✅ 自动配置，不需要手动设置
  self_nav:latest
```

**这些环境变量会在容器启动时自动设置，不需要手动配置。**

### 2. 工作空间环境（需要配置）

但是，**ROS2 工作空间的环境需要手动配置**（每次进入容器都需要）：

```bash
# 进入容器后
source /opt/ros/humble/setup.bash        # ROS2基础环境
source install/setup.bash                # 工作空间环境（如果已编译）
# 或
source /workspace/install/setup.bash     # 根据实际路径调整
```

## 完整流程

### 重新创建容器

```bash
# 停止并删除现有容器
docker stop self_nav rm_vision
docker rm self_nav rm_vision

# 重新创建 self_nav 容器（ROS2通信环境自动配置）
docker run -it --name self_nav \
  --device=/dev/ttyACM0 \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  self_nav:latest

# 重新创建 rm_vision 容器（ROS2通信环境自动配置）
docker run -it --name rm_vision \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  rm_vision:latest
```

### 进入容器后配置工作空间环境

**self_nav 容器：**

```bash
# 进入容器
docker exec -it self_nav bash

# 配置工作空间环境（每次进入都需要）
source /opt/ros/humble/setup.bash
source install/setup.bash
# 或根据实际路径
cd /workspace
source install/setup.bash

# 验证
echo $ROS_DOMAIN_ID
# 应该输出: 0（已自动配置）

ros2 pkg list | grep rm_serial_driver
# 应该能看到包列表
```

**rm_vision 容器：**

```bash
# 进入容器
docker exec -it rm_vision bash

# 配置工作空间环境（每次进入都需要）
source /opt/ros/humble/setup.bash
source install/setup.bash
# 或根据实际路径
cd /workspace
source install/setup.bash

# 验证
echo $ROS_DOMAIN_ID
# 应该输出: 0（已自动配置）
```

## 让工作空间环境自动配置（可选）

### 方法1：修改容器的 .bashrc

```bash
# 进入容器
docker exec -it self_nav bash

# 编辑 .bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /workspace/install/setup.bash' >> ~/.bashrc

# 重新加载
source ~/.bashrc
```

**注意**：这种方法只在当前容器有效，如果删除容器重新创建，需要重新配置。

### 方法2：在 Dockerfile 中配置（推荐）

如果容器有 Dockerfile，可以在 Dockerfile 中添加：

```dockerfile
# 在 Dockerfile 中添加
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /workspace/install/setup.bash' >> ~/.bashrc
```

这样每次创建容器时都会自动配置。

### 方法3：使用启动脚本

创建启动脚本 `entrypoint.sh`：

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
exec "$@"
```

在 docker-compose.yml 中使用：

```yaml
services:
  self_nav:
    # ...
    entrypoint: ["/workspace/entrypoint.sh"]
    command: ["bash"]
```

## 配置清单

### ✅ 自动配置（不需要手动设置）

- `ROS_DOMAIN_ID=0`（通过 `-e ROS_DOMAIN_ID=0`）
- `RMW_IMPLEMENTATION`（如果指定）
- 网络模式（通过 `--network host`）

### ⚠️ 需要手动配置（每次进入容器）

- ROS2 基础环境：`source /opt/ros/humble/setup.bash`
- 工作空间环境：`source install/setup.bash`
- 工作目录：`cd /workspace`（如果需要）

## 快速检查

重新创建容器后，检查配置：

```bash
# 1. 检查 ROS2 通信环境变量（应该已自动配置）
echo $ROS_DOMAIN_ID
# 应该输出: 0

# 2. 检查网络模式
docker inspect self_nav | grep NetworkMode
# 应该输出: "NetworkMode": "host"

# 3. 检查工作空间环境（需要手动source）
ros2 pkg list | grep rm_serial_driver
# 如果找不到，需要 source install/setup.bash
```

## 总结

| 配置项 | 是否需要重新配置 | 说明 |
|--------|-----------------|------|
| **ROS_DOMAIN_ID** | ❌ 不需要 | 通过 `-e` 参数自动配置 |
| **网络模式** | ❌ 不需要 | 通过 `--network host` 自动配置 |
| **ROS2基础环境** | ⚠️ 需要 | 每次进入容器需要 `source /opt/ros/humble/setup.bash` |
| **工作空间环境** | ⚠️ 需要 | 每次进入容器需要 `source install/setup.bash` |

**建议**：将工作空间环境的 source 命令添加到 `.bashrc` 或 Dockerfile 中，实现自动配置。



