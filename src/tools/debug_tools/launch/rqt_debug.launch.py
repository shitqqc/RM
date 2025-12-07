from launch_ros.actions import Node , PushRosNamespace, SetRemap
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace of topic"
    )

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
    ld.add_action(group_actions)
    
    return ld