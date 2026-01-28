#!/usr/bin/env python3
"""
模拟 USB 相机节点
可以模拟左相机或右相机
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
import sys

class MockUsbCamera(Node):
    def __init__(self, camera_name='left', fps=60.0):
        super().__init__(f'mock_usb_{camera_name}_camera')
        
        self.camera_name = camera_name
        
        # 发布器 - 使用命名空间
        namespace = f'/{camera_name}'
        self.image_pub = self.create_publisher(Image, f'{namespace}/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, f'{namespace}/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # 参数
        self.fps = fps
        self.width = 1280
        self.height = 720
        self.frame_count = 0
        
        # 定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        # 颜色标识（左相机用绿色，右相机用红色）
        self.bg_color = (0, 255, 0) if camera_name == 'left' else (0, 0, 255)
        self.text_color = (255, 255, 255)
        
        self.get_logger().info(f'Mock USB {camera_name} camera started at {self.fps} fps')
    
    def publish_frame(self):
        # 创建测试图像（带帧号）
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:, :] = self.bg_color  # 背景色
        
        # 添加文字标识
        text = f"USB {self.camera_name.upper()} Frame: {self.frame_count}"
        cv2.putText(img, text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                    2, self.text_color, 3)
        
        # 添加时间戳显示
        timestamp_text = f"Time: {self.get_clock().now().nanoseconds / 1e9:.3f}s"
        cv2.putText(img, timestamp_text, (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, self.text_color, 2)
        
        # 发布图像
        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = f'{self.camera_name}_camera_optical_frame'
        
        # 发布 camera_info
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = f'{self.camera_name}_camera_optical_frame'
        info_msg.width = self.width
        info_msg.height = self.height
        
        self.image_pub.publish(img_msg)
        self.info_pub.publish(info_msg)
        
        self.frame_count += 1
        
        # 降低日志输出频率到原来的1/10（每600帧输出一次，而不是每60帧）
        if self.frame_count % 600 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

def main(args=None):
    rclpy.init(args=args)
    
    # 从命令行参数获取相机名称和帧率
    camera_name = 'left'
    fps = 60.0
    
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]  # 'left' 或 'right'
    if len(sys.argv) > 2:
        fps = float(sys.argv[2])
    
    node = MockUsbCamera(camera_name=camera_name, fps=fps)
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

