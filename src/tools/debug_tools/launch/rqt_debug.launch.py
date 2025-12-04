from launch_ros.actions import Node , PushRosNamespace, SetRemap
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    # use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace of topic"
    )
    
    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     "use_sim_time",
    #     default_value="false",
    #     description="Use simulation (Gazebo) clock if true"
    # )

    rqt_gui_node = Node(
        package="rqt_gui", executable="rqt_gui", name="rqt_gui", 
    )
    
    group_actions = GroupAction([
        PushRosNamespace(namespace=namespace),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),    
        rqt_gui_node,
    ])
    
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(group_actions)
    
    return ld