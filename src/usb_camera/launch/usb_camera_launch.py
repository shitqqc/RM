from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('usb_camera')
    
    # 声明合并后的配置文件参数
    common_config_arg = DeclareLaunchArgument(
        'common_config',
        default_value=os.path.join(pkg_share, 'config', 'cameras_params.yaml'),
        description='Path to common cameras config file'
    )
    
    camera1_device_arg = DeclareLaunchArgument(
        'camera1_device',
        default_value='/dev/video4',
        description='Device path for camera 1'
    )
    
    camera2_device_arg = DeclareLaunchArgument(
        'camera2_device',
        default_value='/dev/video2',
        description='Device path for camera 2'
    )
    
    left_camera_component = ComposableNode(
        package='usb_camera',
        plugin='usb_camera::UsbCameraNode',
        name='usb_camera',
        namespace='left',
        parameters=[
            # 只传递配置文件，不单独传递参数
            LaunchConfiguration('common_config'),
            # 通过命名空间指定使用左相机的配置部分
            {'camera_name': 'left_camera'}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info')
        ]
    )

    right_camera_component = ComposableNode(
        package='usb_camera',
        plugin='usb_camera::UsbCameraNode',
        name='usb_camera',
        namespace='right',
        parameters=[
            #同上
            LaunchConfiguration('common_config'),
            {'camera_name': 'right_camera'}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info')
        ]
    )
    
    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            left_camera_component,
            right_camera_component
        ],
        output='screen'
    )

    return LaunchDescription([
        common_config_arg,
        camera1_device_arg,
        camera2_device_arg,
        camera_container
    ])