## set OpenVINO_LIBRARIES
set(OpenVINO_LIBRARIES openvino::runtime)

cmake_minimum_required(VERSION 3.8)
project(armor_detector)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
set(G2O_DIR "/usr/local/lib/cmake/g2o")

set(EXTERNAL_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${G2O_LIBRARIES})


find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${OpenVINO_INCLUDE_DIRS} 
)

target_link_libraries(${PROJECT_NAME}
${OpenVINO_LIBRARIES} 
${OpenCV_LIBS}
g2o_core
g2o_stuff
g2o_solver_csparse
g2o_types_sba
g2o_types_slam3d
g2o_solver_dense
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
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

ament_auto_package(
  INSTALL_TO_SHARE
  models
  #launch
)