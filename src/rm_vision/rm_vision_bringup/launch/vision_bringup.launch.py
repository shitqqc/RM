import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, armor_tracker_node
    from launch_ros.actions import Node
    from launch.actions import TimerAction, Shutdown, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch import LaunchDescription

    camera_sync_dir = get_package_share_directory('camera_sync')
    multi_camera_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_sync_dir, 'launch', 'multi_camera_vision.launch.py')
        )
    )

    # 串口
    if launch_params['virtual_serial']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args',],
        )
    else:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[node_params],
            on_exit=Shutdown(),
            ros_arguments=['--ros-args', '--log-level',
                    'serial_driver:='+launch_params['serial_log_level']],
        )

    
    rm_planner_node = Node(
        package='rm_planner',
        executable='rm_planner_node',
        name='rm_planner',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--ros-args', '--log-level',
                       'rm_planner:='+launch_params['planner_log_level']],
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_armor_tracker_node = TimerAction(
        period=2.0,
        actions=[armor_tracker_node],
    )
    delay_planner_node = TimerAction(
        period=2.5,
        actions=[rm_planner_node],
    )
    # delay_buff_tracker_node = TimerAction(
    #     period=2.0,
    #     actions=[buff_tracker_node],
    # )
    
    # delay_auto_record_node = TimerAction(
    #     period=2.5,
    #     actions=[auto_record_node],
    # )

    return LaunchDescription([
        robot_state_publisher,
        multi_camera_vision,
        delay_serial_node,
        delay_armor_tracker_node,
        delay_planner_node
       # delay_buff_tracker_node,
       # delay_auto_record_node
    ])
