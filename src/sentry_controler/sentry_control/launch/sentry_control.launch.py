import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    
    param_rewrites = {"use_sim_time": use_sim_time}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_rewrites,
            convert_types=True
        ),
        allow_substs=True
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use (Ign) gazebo sim time if true"
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace"
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory("sentry_control"), "config", "sentry_control.yaml"),
        description="Default config file path"
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level"
    )
    
    sentry_control_node = Node(
        package="sentry_control",
        executable="sentry_control_node",
        name="sentry_control",
        namespace=namespace,
        parameters=[
            configured_params
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen"
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(sentry_control_node)
    
    return ld