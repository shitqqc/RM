cmake_minimum_required(VERSION 3.10)
project(rm_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)
add_subdirectory(include/rm_planner/tinympc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# 对于 Humble，使用 ament_target_dependencies 更可靠
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}
  tinympcstatic
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::RMPlannerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

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

ament_auto_package()