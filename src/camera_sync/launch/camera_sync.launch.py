import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    camera_sync_dir = get_package_share_directory('camera_sync')
    
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(camera_sync_dir, 'config', 'camera_sync_params.yaml'),
        description='Full path to camera sync parameters file'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level'
    )
    
    camera_sync_node = Node(
        package='camera_sync',
        executable='camera_sync_node',
        name='camera_sync_node',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(camera_sync_node)
    
    return ld