"""模拟相机节点"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_sync_dir = get_package_share_directory('camera_sync')
    
    # Launch 参数
    hik_fps_arg = DeclareLaunchArgument(
        'hik_fps',
        default_value='200.0',
        description='HIK相机模拟帧率 (Hz)'
    )
    
    usb_fps_arg = DeclareLaunchArgument(
        'usb_fps',
        default_value='60.0',
        description='USB相机模拟帧率 (Hz)'
    )
    
    max_time_diff_arg = DeclareLaunchArgument(
        'max_time_diff_ms',
        default_value='200.0',  # 增加到200ms以提高同步率到85%
        description='最大时间差阈值 (ms)'
    )
    
    # 模拟 HIK 相机节点
    mock_hik_node = Node(
        package='camera_sync',
        executable='mock_hik_camera.py',
        name='mock_hik_camera',
        parameters=[{
            'fps': LaunchConfiguration('hik_fps')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 模拟 USB 左相机节点
    mock_usb_left_node = Node(
        package='camera_sync',
        executable='mock_usb_camera.py',
        name='mock_usb_left_camera',
        arguments=['left', LaunchConfiguration('usb_fps')],
        output='screen',
        emulate_tty=True
    )
    
    # 模拟 USB 右相机节点
    mock_usb_right_node = Node(
        package='camera_sync',
        executable='mock_usb_camera.py',
        name='mock_usb_right_camera',
        arguments=['right', LaunchConfiguration('usb_fps')],
        output='screen',
        emulate_tty=True
    )
    
    # 同步节点参数文件
    sync_params_file = os.path.join(camera_sync_dir, 'config', 'camera_sync_params.yaml')
    
    # 相机同步节点
    camera_sync_node = Node(
        package='camera_sync',
        executable='camera_sync_node',
        name='camera_sync_node',
        parameters=[
            sync_params_file,
            {'max_time_diff_ms': LaunchConfiguration('max_time_diff_ms')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    # 图像查看器（可选，用于可视化）
    # 需要安装: sudo apt install ros-<distro>-rqt-image-view
    # image_view_hik = Node(
    #     package='rqt_image_view',
    #     executable='rqt_image_view',
    #     name='image_view_hik',
    #     arguments=['/sync/hik/image'],
    #     output='screen'
    # )
    
    ld = LaunchDescription()
    ld.add_action(hik_fps_arg)
    ld.add_action(usb_fps_arg)
    ld.add_action(max_time_diff_arg)
    ld.add_action(mock_hik_node)
    ld.add_action(mock_usb_left_node)
    ld.add_action(mock_usb_right_node)
    ld.add_action(camera_sync_node)
    # ld.add_action(image_view_hik)  # 取消注释以启用图像查看
    
    return ld

