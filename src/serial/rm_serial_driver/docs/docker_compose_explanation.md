# Docker Compose 配置说明

## 永久生效性

**是的，Docker Compose 方法是永久生效的！**

### 为什么是永久的？

1. **配置文件持久化**：`docker-compose.yml` 文件保存在宿主机上，配置不会丢失
2. **每次启动自动应用**：使用 `docker-compose up` 启动时，会自动应用配置文件中的所有设置
3. **环境变量持久化**：`environment` 部分的配置会写入容器配置，每次启动都生效

### 对比三种方法

| 方法 | 持久性 | 说明 |
|------|--------|------|
| **方法1：临时设置** | ❌ 临时 | 只在当前bash会话有效，容器重启后失效 |
| **方法2：重新创建容器** | ✅ 永久 | 配置写入容器创建命令，但需要记住命令 |
| **方法3：Docker Compose** | ✅ 永久 | 配置保存在文件中，最易管理和维护 |

## Docker Compose 的优势

### 1. 配置持久化
- 所有配置保存在 `docker-compose.yml` 文件中
- 可以版本控制（git）
- 易于分享和备份

### 2. 一键启动/停止
```bash
# 启动所有容器
docker-compose up -d

# 停止所有容器
docker-compose stop

# 停止并删除容器
docker-compose down
```

### 3. 统一管理
- 两个容器的配置在一个文件中
- 易于修改和维护
- 可以设置依赖关系

### 4. 环境变量管理
- 可以在 `.env` 文件中管理环境变量
- 支持环境变量替换
- 更安全（不暴露在命令行）

## 完整示例

### docker-compose.yml

```yaml
version: '3.8'

services:
  self_nav:
    container_name: self_nav
    image: self_nav:latest
    network_mode: host                    # 永久配置：使用主机网络
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
    environment:
      - ROS_DOMAIN_ID=0                  # 永久配置：ROS2域名ID
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    volumes:
      - /home/rm/self_nav:/workspace
    stdin_open: true
    tty: true
    restart: unless-stopped              # 容器自动重启

  rm_vision:
    container_name: rm_vision
    image: rm_vision:latest
    network_mode: host                    # 永久配置：使用主机网络
    environment:
      - ROS_DOMAIN_ID=0                  # 永久配置：相同的ROS2域名ID
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    volumes:
      - /home/rm/rm_vision:/workspace
    stdin_open: true
    tty: true
    restart: unless-stopped              # 容器自动重启
```

### 使用步骤

**1. 创建配置文件**

```bash
# 在宿主机上创建 docker-compose.yml
cd /home/rm
nano docker-compose.yml
# 粘贴上面的配置内容
```

**2. 停止并删除现有容器**

```bash
docker stop self_nav rm_vision
docker rm self_nav rm_vision
```

**3. 使用 docker-compose 启动**

```bash
# 启动容器（后台运行）
docker-compose up -d

# 查看日志
docker-compose logs -f

# 进入容器
docker-compose exec self_nav bash
docker-compose exec rm_vision bash
```

**4. 验证配置**

```bash
# 检查容器配置
docker-compose config

# 检查环境变量
docker-compose exec self_nav env | grep ROS_DOMAIN_ID
docker-compose exec rm_vision env | grep ROS_DOMAIN_ID
```

## 持久性验证

### 测试1：重启容器

```bash
# 重启容器
docker-compose restart

# 检查环境变量（应该仍然是 ROS_DOMAIN_ID=0）
docker-compose exec self_nav env | grep ROS_DOMAIN_ID
```

### 测试2：删除并重新创建

```bash
# 删除容器
docker-compose down

# 重新创建（配置仍然生效）
docker-compose up -d

# 检查环境变量（应该仍然是 ROS_DOMAIN_ID=0）
docker-compose exec self_nav env | grep ROS_DOMAIN_ID
```

## 高级配置：使用 .env 文件

### 创建 .env 文件

```bash
# .env
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SERIAL_DEVICE=/dev/ttyACM0
```

### 修改 docker-compose.yml

```yaml
version: '3.8'

services:
  self_nav:
    container_name: self_nav
    image: self_nav:latest
    network_mode: host
    devices:
      - ${SERIAL_DEVICE}:/dev/ttyACM0
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
      - RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
    # ...
```

这样可以通过修改 `.env` 文件来改变配置，而不需要修改 `docker-compose.yml`。

## 总结

**Docker Compose 方法的特点**：

✅ **永久生效**：配置保存在文件中，每次启动都应用  
✅ **易于管理**：所有配置在一个文件中  
✅ **版本控制**：可以提交到git  
✅ **易于修改**：修改配置文件后重新启动即可  
✅ **自动化**：可以设置自动重启、依赖关系等  

**推荐使用 Docker Compose**，特别是对于需要长期运行的生产环境。



