#!/bin/bash
# 检查话题和统计信息的脚本
# 在另一个终端运行，用于监控测试状态

echo "=========================================="
echo "相机同步节点监控工具"
echo "=========================================="
echo ""

# Source 环境
if [ -d "install" ]; then
    source install/setup.bash
elif [ -d "../install" ]; then
    source ../install/setup.bash
fi

echo "1. 检查所有话题..."
echo "----------------------------------------"
ros2 topic list
echo ""

echo "2. 检查话题频率（运行5秒）..."
echo "----------------------------------------"
echo "HIK原始图像频率:"
timeout 5 ros2 topic hz /image_raw 2>/dev/null || echo "  未检测到话题"
echo ""

echo "HIK同步后图像频率:"
timeout 5 ros2 topic hz /sync/hik/image 2>/dev/null || echo "  未检测到话题"
echo ""

echo "USB左相机频率:"
timeout 5 ros2 topic hz /left/image_raw 2>/dev/null || echo "  未检测到话题"
echo ""

echo "USB右相机频率:"
timeout 5 ros2 topic hz /right/image_raw 2>/dev/null || echo "  未检测到话题"
echo ""

echo "3. 查看同步节点日志（最后10条）..."
echo "----------------------------------------"
ros2 topic echo /rosout --once 2>/dev/null | grep -i "sync\|stats\|received" | tail -10 || echo "  未找到相关日志"
echo ""

echo "4. 检查节点状态..."
echo "----------------------------------------"
ros2 node list
echo ""

echo "=========================================="
echo "提示: 使用以下命令查看实时日志:"
echo "  ros2 topic echo /rosout | grep -i sync"
echo "=========================================="












