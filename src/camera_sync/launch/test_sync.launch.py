"""测试同步节点的 launch 文件"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    camera_sync_dir = get_package_share_directory('camera_sync')
    usb_camera_dir = get_package_share_directory('usb_camera')
    
    # 模拟 HIK 相机
    mock_hik_node = Node(
        package='camera_sync',
        executable='mock_hik_camera.py',
        name='mock_hik_camera',
        output='screen'
    )
    
    # USB 相机参数文件
    usb_params_file = os.path.join(usb_camera_dir, 'config', 'usb_cameras.yaml')
    
    # USB 左相机
    usb_left_container = ComposableNodeContainer(
        name='usb_camera_left_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_camera',
                plugin='usb_camera::UsbCameraNode',
                name='usb_camera_left',
                parameters=[usb_params_file],
            )
        ],
        output='screen'
    )
    
    # USB 右相机
    usb_right_container = ComposableNodeContainer(
        name='usb_camera_right_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_camera',
                plugin='usb_camera::UsbCameraNode',
                name='usb_camera_right',
                parameters=[usb_params_file],
            )
        ],
        output='screen'
    )
    
    # 同步节点
    sync_params_file = os.path.join(camera_sync_dir, 'config', 'camera_sync_params.yaml')
    camera_sync_node = Node(
        package='camera_sync',
        executable='camera_sync_node',
        name='camera_sync_node',
        parameters=[sync_params_file],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(mock_hik_node)
    ld.add_action(usb_left_container)
    ld.add_action(usb_right_container)
    ld.add_action(camera_sync_node)
    
    return ld
