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

    rviz2_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", 
    )
    
    group_actions = GroupAction([
        PushRosNamespace(namespace=namespace),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),    
        rviz2_node,
    ])
    
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(group_actions)
    
    return ld