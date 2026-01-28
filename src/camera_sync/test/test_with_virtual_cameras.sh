#!/bin/bash
# 使用虚拟相机测试多相机视觉处理系统

set -e

echo "=========================================="
echo "虚拟相机测试 - 多相机视觉处理系统"
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

echo ""
echo "选择测试模式："
echo "1. 仅测试相机同步（不启动检测器）"
echo "2. 完整测试（相机同步 + 检测器）"
read -p "请选择 (1/2): " choice

case $choice in
    1)
        echo ""
        echo "启动相机同步测试（不启动检测器）..."
        ros2 launch camera_sync test_camera_sync.launch.py \
            hik_fps:=200.0 \
            usb_fps:=60.0 \
            max_time_diff_ms:=200.0
        ;;
    2)
        echo ""
        echo "启动完整测试（相机同步 + 检测器）..."
        echo "注意：需要模型文件，如果模型不存在可能会报错"
        ros2 launch camera_sync test_multi_camera_vision.launch.py \
            hik_fps:=200.0 \
            usb_fps:=60.0 \
            max_time_diff_ms:=200.0 \
            enable_detector:=true
        ;;
    *)
        echo "无效选择，退出"
        exit 1
        ;;
esac








