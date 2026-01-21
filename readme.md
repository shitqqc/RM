# 华南师范大学PIONEER战队2026赛季导航开发仓库

基于深北莫polorbear战队pb2025_sentry_nav开发，原仓库地址：
<https://github.com/polarbear-robotics/pb2025_sentry_nav>
配套仿真地址：
<https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator>

## 目前完成功能

1. 指定角度的pid底盘控制，限制速度下通过指定区域
2. 自动录包

## Usage

### Start

拉取仓库

```bash
git clone git@github.com:SCNU-PIONEER/PIONEER_sentry_nav.git
cd PIONEER_sentry_nav/
```

安装依赖

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

编译

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> 关于编译参数：
>-DCMAKE_BUILD_TYPE=Release：编译时使用Release模式，可以加快编译速度，并优化代码运行效率。 （必须，否则重定位无法正常运行）
>--symlink-install：使用符号链接进行安装，可以避免重复编译和安装，节省存储空间和编译时间。  （加上后更改config时无须重新编译）

### Run

>参考深圳北理莫斯科大学北极熊战队pb2025_sentry_nav仓库

#### 仿真

```bash
#导航模式
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2026 \
slam:=False
```

```bash
#建图模式
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

保存格栅地图  
`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=< YOUR_NAMESPACE >`
>地图保存在工作空间目录下，要使用需手动将.png和.yaml文件复制到`...ws/src/pb2025_sentry_nav/pb2025_nav_bringup/map`对应文件夹.
>完成建图后生成的.pcd文件默认在point_lid/PCD文件夹下，默认名称为`scans.pcd`，需手动复制到`...ws/src/pb2025_sentry_nav/pb2025_nav_bringup/PCD`对应文件夹下，并修改文件名称.
>将点云和栅格地图保存到`pb2025_nav_bringup`后需要重新编译以将文件安装到`install`文件夹下.

#### 实车

```bash
#建图模式
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

保存地图方式同仿真

```bash
#导航模式
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True
```

### 录包

每次启动程序5s后，会自动开始录包。录包保存在`..ws/bag/`下，包名称为bag_YYYYMMDD_HHMMSS.bag，其中YYYYMMDD_HHMMSS为当前时间。
如需修改录包的话题，请修改`src/pb2025_sentry_nav/pb2025_nav_bringup/launch/record_bag.launch.py`文件中`record_topics`变量。录包默认需要`namesoace`，如不使用`namespace`，请将`record_topics`变量中的`/namespace/TOPIC_NAME`改为`/TOPIC_NAME`。

## TODO

1. 增加选择是否有先验点与的参数
2. 将现有的`sentry_controler`下的功能包直接与控制器插件整合为一个插件
3. 无先验点云导航下的建图
