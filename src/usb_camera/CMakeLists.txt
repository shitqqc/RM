cmake_minimum_required(VERSION 3.8)
project(usb_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

add_library(usb_camera_node SHARED src/usb_camera_node.cpp)

ament_target_dependencies(usb_camera_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  camera_info_manager
)

rclcpp_components_register_nodes(usb_camera_node "usb_camera::UsbCameraNode")

# 关键：正确安装组件库到lib目录
install(TARGETS usb_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()