cmake_minimum_required(VERSION 3.8)
## set OpenVINO_LIBRARIES
set(OpenVINO_LIBRARIES openvino::runtime)

project(buff_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC 
${OpenCV_INCLUDE_DIRS}
${OpenVINO_INCLUDE_DIRS} 
)
target_link_libraries(${PROJECT_NAME} 
${OpenCV_LIBS}
${OpenVINO_LIBRARIES} )

rclcpp_components_register_node(${PROJECT_NAME}
PLUGIN rm_buff::BuffDetectorNode
EXECUTABLE buff_detector_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  )