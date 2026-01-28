#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge

class MockHikCamera(Node):
    def __init__(self):
        super().__init__('mock_hik_camera')
        
        # 声明参数
        self.declare_parameter('fps', 200.0)  # 默认200Hz，模拟HIK相机高帧率
        
        # 发布器
        self.image_pub = self.create_publisher(Image, '/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # 参数
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.width = 1440
        self.height = 1080
        self.frame_count = 0
        
        # 定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info(f'Mock HIK camera started at {self.fps} fps')
    
    def publish_frame(self):
        # 创建测试图像（带帧号）
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:, :] = (100, 150, 200)  # 浅蓝色背景
        
        # 添加文字
        text = f"HIK Frame: {self.frame_count}"
        cv2.putText(img, text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                    2, (255, 255, 255), 3)
        
        # 发布图像
        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera_optical_frame'
        
        # 发布 camera_info
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = 'camera_optical_frame'
        info_msg.width = self.width
        info_msg.height = self.height
        
        self.image_pub.publish(img_msg)
        self.info_pub.publish(info_msg)
        
        self.frame_count += 1
        
        # 降低日志输出频率到原来的1/10（每300帧输出一次，而不是每30帧）
        if self.frame_count % 300 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

def main(args=None):
    rclpy.init(args=args)
    node = MockHikCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
