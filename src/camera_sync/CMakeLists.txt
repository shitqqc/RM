cmake_minimum_required(VERSION 3.8)
project(camera_sync)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)

# Camera sync node library
add_library(${PROJECT_NAME} SHARED
  src/camera_sync_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
  image_transport
  cv_bridge
  camera_info_manager
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "camera_sync::CameraSyncNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装 launch 和 config 文件
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 安装测试脚本
install(PROGRAMS
  test/mock_hik_camera.py
  test/mock_usb_camera.py
  test/quick_test.sh
  test/check_topics.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
