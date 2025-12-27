# 华南师范大学PIONEER战队2026赛季导航开发仓库

| 基于深北莫polorbear战队pb2025_sentry_nav开发

## 目前完成功能

1. 指定角度的pid底盘控制，限制速度下通过指定区域
2. 自动录包，保存在`...ws/bags`下

## Usage

安装依赖
'''bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
'''

编译
‘’‘bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
'''

|  关于编译参数：
|  -DCMAKE_BUILD_TYPE=Release：编译时使用Release模式，可以加快编译速度，并优化代码运行效率。 （必须，否则重定位无法正常运行）
|  --symlink-install：使用符号链接进行安装，可以避免重复编译和安装，节省存储空间和编译时间。  （加上后更改config时无须重新编译）
