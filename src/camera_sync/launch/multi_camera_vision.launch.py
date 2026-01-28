import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 获取各包路径
    hik_camera_dir = get_package_share_directory('hik_camera')
    usb_camera_dir = get_package_share_directory('usb_camera')
    camera_sync_dir = get_package_share_directory('camera_sync')
    # 使用rm_vision_bringup的配置文件（如果没有armor_detector的独立配置文件）
    vision_bringup_dir = get_package_share_directory('rm_vision_bringup')
    
    # Launch参数
    use_pose_estimation_arg = DeclareLaunchArgument(
        'use_pose_estimation',
        default_value='true',
        description='是否使用姿态估计（HIK相机使用，USB相机不使用）'
    )
    
    # HIK 相机
    hik_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hik_camera_dir, 'launch', 'hik_camera_launch.py')
        )
    )
    
    # USB 相机
    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb_camera_dir, 'launch', 'usb_camera_launch.py')
        )
    )
    
    # 相机同步节点
    camera_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_sync_dir, 'launch', 'camera_sync.launch.py')
        )
    )
    
    # HIK相机完整检测器（包含姿态估计）
    # 直接订阅同步后的HIK图像，处理后的debug图像也发布到对应话题
    hik_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='hik_armor_detector',
        parameters=[
            os.path.join(vision_bringup_dir, 'config', 'node_params.yaml'),
            {
                'enable_pose_estimation': True,  # 启用姿态估计
                'use_ba': True,  # 使用BA优化
                'target_frame': 'odom',
            }
        ],
        remappings=[
            ('/image_raw', '/sync/hik/image'),  # 订阅同步后的HIK图像
            ('/camera_info', '/sync/hik/camera_info'),
            ('/detector/armors', '/hik/detector/armors'),
            ('/detector/binary_img', '/hik/detector/binary_img'),  # debug话题
            ('/detector/result_img', '/hik/detector/result_img'),  # debug话题
            ('/detector/marker', '/hik/detector/marker'),  # marker话题
        ],
        output='screen',
        emulate_tty=True
    )
    
    # USB左相机检测器（仅数字分类，无姿态估计）
    # 直接订阅同步后的USB左图像
    usb_left_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='usb_left_armor_detector',
        parameters=[
            os.path.join(vision_bringup_dir, 'config', 'node_params.yaml'),
            {
                'enable_pose_estimation': False,  # 禁用姿态估计，只做检测和分类
            }
        ],
        remappings=[
            ('/image_raw', '/sync/usb_left/image'),  # 订阅同步后的USB左图像
            ('/camera_info', '/sync/usb_left/camera_info'),
            ('/detector/armors', '/usb_left/detector/armors'),
            ('/detector/binary_img', '/usb_left/detector/binary_img'),  # debug话题
            ('/detector/result_img', '/usb_left/detector/result_img'),  # debug话题
        ],
        output='screen',
        emulate_tty=True
    )
    
    # USB右相机检测器（仅数字分类，无姿态估计）
    # 直接订阅同步后的USB右图像
    usb_right_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='usb_right_armor_detector',
        parameters=[
            os.path.join(vision_bringup_dir, 'config', 'node_params.yaml'),
            {
                'enable_pose_estimation': False,  # 禁用姿态估计，只做检测和分类
            }
        ],
        remappings=[
            ('/image_raw', '/sync/usb_right/image'),  # 订阅同步后的USB右图像
            ('/camera_info', '/sync/usb_right/camera_info'),
            ('/detector/armors', '/usb_right/detector/armors'),
            ('/detector/binary_img', '/usb_right/detector/binary_img'),  # debug话题
            ('/detector/result_img', '/usb_right/detector/result_img'),  # debug话题
        ],
        output='screen',
        emulate_tty=True
    )
    
    ld = LaunchDescription()
    ld.add_action(use_pose_estimation_arg)
    ld.add_action(hik_camera_launch)
    ld.add_action(usb_camera_launch)
    ld.add_action(camera_sync_launch)
    ld.add_action(hik_detector_node)
    ld.add_action(usb_left_detector_node)
    ld.add_action(usb_right_detector_node)
    
    return ld

