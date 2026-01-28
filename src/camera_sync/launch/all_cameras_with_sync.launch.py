"""启动所有相机和同步节点"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取各包路径 - 修改这里的包名
    hik_camera_dir = get_package_share_directory('hik_camera')  # 改成 hik_camera
    usb_camera_dir = get_package_share_directory('usb_camera')
    camera_sync_dir = get_package_share_directory('camera_sync')
    
    # HIK 相机
    hik_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hik_camera_dir, 'launch', 'hik_camera_launch.py')
        )
    )
    
    # USB 相机
    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb_camera_dir, 'launch', 'usb_camera_launch.py')
        )
    )
    
    # 相机同步节点
    camera_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_sync_dir, 'launch', 'camera_sync.launch.py')
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(hik_camera_launch)
    ld.add_action(usb_camera_launch)
    ld.add_action(camera_sync_launch)
    
    return ld
