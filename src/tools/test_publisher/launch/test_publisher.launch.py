import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use (Ign) gazebo sim time if true"
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace"
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory("chassis_angle"), "config", "chassis_angle.yaml"),
        description="Default config file path"
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level"
    )
    
    chassis_angle_node = Node(
        package="test_publisher",
        executable="test_publisher_node",
        name="test_publisher",
        namespace=namespace,
        parameters=[
            {"use_sim_time": True}
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(chassis_angle_node)
    
    return ld