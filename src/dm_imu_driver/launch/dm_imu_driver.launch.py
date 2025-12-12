import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory('dm_imu_driver'), 'config', 'node_params.yaml')

    launch_config = os.path.join(
        get_package_share_directory('dm_imu_driver'), 'config', 'launch_params.yaml')
    
    # 预先读取YAML文件获取日志级别 #
    log_level_ = 'INFO' # 默认值
    try:
        with open(launch_config, 'r') as file:
            data = yaml.safe_load(file)
            log_level_ = data.get('dm_imu_driver_log_level')
            print(f"[INFO] [launch] Loaded log level from YAML: {log_level_}")
            log_level = log_level_
    except Exception as e:
        print(f"[Warning] [launch]: Could not read log level from YAML: {e}")
    ############################
    
    imu_driver_node = Node(
        package='dm_imu_driver',
        executable='dm_imu_driver_node',
        # namespace='',
        name='dm_imu_driver',
        output='both',
        emulate_tty=True,
        # on_exit=Shutdown(),
        parameters=[node_params],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        imu_driver_node
    ])