from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation_time if ture"
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace of topic"
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level"
    )
    
    group_actions = GroupAction([
        PushRosNamespace(namespace=namespace),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
    
        Node(
            package="gimbal_spin",
            executable="gimbal_spin_node",
            name="gimbal_spin",
            parameters=[{"use_sim_time":use_sim_time}],
            arguments=['--ros-args','--log-level',log_level],
        )
    ])
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(group_actions)
    
    return ld
