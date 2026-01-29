import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    
    param_rewrites = {"use_sim_time": use_sim_time}
    
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("")},
        condition=LaunchConfigurationEquals("namespace", ""),
    )

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=LaunchConfigurationNotEquals("namespace", ""),
    )
    
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
        default_value="True",
        description="Use (Ign) gazebo sim time if true"
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace"
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "simulation", "nav2_params.yaml"),
        description="Default config file path"
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="log level"
    )
    
    # Get rm_decision_cpp package directory
    try:
        rm_decision_cpp_dir = get_package_share_directory("rm_decision_cpp")
    except:
        # 如果包还没有安装，使用源码路径
        import pathlib
        self_nav_src = pathlib.Path(__file__).parent.parent.parent.parent.parent
        rm_decision_cpp_dir = str(self_nav_src / "rm_decision_cpp")
    
    Node_group = GroupAction([
        Node(
            package="chassis_angle",
            executable="chassis_angle_node",
            name="chassis_angle",
            parameters=[
                configured_params,
                {"use_sim_time": use_sim_time}
            ],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static")
                ],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen"
        ),
        # 使用 SCURM_SentryNavigation 的决策框架替代 sentry_control
        Node(
            package="rm_decision_cpp",
            executable="tree_exec_node",
            name="tree_exec",
            parameters=[
                os.path.join(rm_decision_cpp_dir, "config", "node_params.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    # 使用全向装甲测试行为树
                    "tree_xml_file": os.path.join(rm_decision_cpp_dir, "behavior_tree", "test_omni_armor.xml"),
                    "tree_node_model_export_path": os.path.join(rm_decision_cpp_dir, "behavior_tree", "tree_nodes.xml"),
                }
            ],
            remappings=[
                # 导航相关话题映射（根据命名空间调整）
                ("navigate_to_pose", (namespace, "/", "navigate_to_pose")),
                # 游戏状态话题
                ("/game_state", (namespace, "/", "game_state")),
                # 目标跟踪话题
                ("/tracker/target", (namespace, "/", "tracker", "/", "target")),
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen"
        ),
    ])
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(Node_group)
    
    return ld