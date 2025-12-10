import os
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    delay = float(LaunchConfiguration('delay').perform(context))
    output_dir = LaunchConfiguration('output_dir').perform(context)

    # 自动生成时间戳文件夹
    timestamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    bag_output_path = f"{output_dir}/bag_{timestamp}"

    # 录制话题（此时 namespace 是字符串，可以直接拼）
    record_topics = [
        f"/{namespace}/tf",
        f"/{namespace}/tf_static",
        f"/{namespace}/livox/imu",
        f"/{namespace}/livox/lidar",
    ]

    record_cmd = ['ros2', 'bag', 'record', '-o', bag_output_path] + record_topics

    rosbag_record = ExecuteProcess(
        cmd=record_cmd,
        output='screen'
    )

    delayed_rosbag = TimerAction(
        period=delay,
        actions=[
            LogInfo(msg=f"Starting ros2 bag recording for namespace: {namespace}"),
            rosbag_record
        ]
    )

    return [delayed_rosbag]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot1', description='Namespace of topics to record'),
        DeclareLaunchArgument('delay', default_value='5.0', description='Seconds to wait before starting ros2 bag'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(get_package_share_directory('pb2025_nav_bringup'), '..', '..', '..', '..', 'bag'), description='Directory to save ros2 bag files'),
        LogInfo(msg="Launching auto ros2 bag recorder..."),
        OpaqueFunction(function=launch_setup)
    ])
