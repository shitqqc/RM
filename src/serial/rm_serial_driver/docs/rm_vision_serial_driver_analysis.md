# rm_vision 中 rm_serial_driver 功能分析

## rm_vision 中 rm_serial_driver 的功能

### 1. 发送功能（可以被串口桥接节点替代）
- ✅ 订阅 `armor_solver/cmd_gimbal` 话题
- ✅ 发送 `GimbalCmd` 到串口
- **状态**：串口桥接节点已实现此功能

### 2. 接收功能（包含重要业务逻辑）

接收串口数据包（0x5A）后，执行以下操作：

#### 2.1 参数设置
- 根据 `detect_color` 设置 `armor_detector` 的参数
- 调用 `detector_param_client_->set_parameters()`

#### 2.2 服务调用
- `reset_tracker`：调用 `/tracker/reset` 服务
- `change_target`：调用 `/tracker/change` 服务

#### 2.3 话题发布
- `/task_mode`：根据 `game_time` 和 `task_mode` 判断任务模式（aim/small_buff/large_buff/auto）
- `/record_controller`：发布录制控制命令（start/stop）
- `/aiming_point`：发布可视化Marker（瞄准点）
- `/latency`：发布延迟信息

#### 2.4 TF发布
- 发布 `odom -> gimbal_link` 的TF变换（从串口接收的roll/pitch/yaw）

## 结论

### 选项1：保留 rm_vision 中的 rm_serial_driver（推荐用于过渡）

**优点**：
- 业务逻辑保持不变
- 不需要修改现有代码

**缺点**：
- 需要修改为不直接访问串口，而是订阅串口桥接节点发布的话题
- 两个串口驱动节点，架构不够清晰

**实现方式**：
- rm_vision 中的 rm_serial_driver 不再直接访问串口
- 订阅串口桥接节点发布的视觉状态话题
- 处理业务逻辑（参数设置、服务调用、话题发布）

### 选项2：完全移除 rm_vision 中的 rm_serial_driver（推荐长期方案）

**优点**：
- 架构清晰，只有一个串口管理节点
- 统一管理，避免冲突

**缺点**：
- 需要在串口桥接节点中实现所有业务逻辑
- 需要修改视觉模块的代码

**实现方式**：
- 在串口桥接节点中实现所有业务逻辑
- 串口桥接节点发布：
  - `/task_mode`
  - `/record_controller`
  - `/aiming_point`
  - TF变换
  - 调用视觉模块的服务

## 推荐方案

### 短期方案（快速实现）

**保留 rm_vision 中的 rm_serial_driver，但修改为不直接访问串口**：

1. 修改 rm_vision 中的 rm_serial_driver，移除串口访问代码
2. 订阅串口桥接节点发布的视觉状态话题
3. 保留所有业务逻辑处理

### 长期方案（架构优化）

**完全移除 rm_vision 中的 rm_serial_driver**：

1. 在串口桥接节点中实现所有业务逻辑
2. 串口桥接节点直接调用视觉模块的服务和发布话题
3. rm_vision 容器中不再需要串口驱动

## 当前状态

串口桥接节点目前**只实现了数据转发**，没有实现业务逻辑处理。

如果要完全替代 rm_vision 中的 rm_serial_driver，需要在串口桥接节点中添加：
- 参数客户端（设置detect_color）
- 服务客户端（reset_tracker, change_target）
- 任务模式判断逻辑
- TF发布
- 可视化Marker发布



