#!/bin/bash
# 快速测试脚本 - 测试相机同步节点

set -e

echo "=========================================="
echo "相机同步节点快速测试"
echo "=========================================="

# 检查是否在正确的目录
if [ ! -d "install" ]; then
    echo "错误: 请在 ROS2 工作空间根目录运行此脚本"
    echo "当前目录: $(pwd)"
    exit 1
fi

# Source 环境
echo "正在加载 ROS2 环境..."
source install/setup.bash

# 检查话题是否已存在
echo ""
echo "检查现有话题..."
EXISTING_TOPICS=$(ros2 topic list 2>/dev/null | wc -l)
if [ "$EXISTING_TOPICS" -gt 0 ]; then
    echo "警告: 检测到现有话题，可能已有节点在运行"
    echo "建议先停止所有相关节点，然后重新运行测试"
    read -p "是否继续? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "启动测试 launch 文件..."
echo "按 Ctrl+C 停止测试"
echo ""

# 启动测试（使用优化后的参数以提高同步率）
ros2 launch camera_sync test_camera_sync.launch.py \
    hik_fps:=200.0 \
    usb_fps:=60.0 \
    max_time_diff_ms:=200.0

