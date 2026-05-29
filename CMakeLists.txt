cmake_minimum_required(VERSION 3.0.2)
project(scope)

## ==============================
## C++ standard
## ==============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

## ==============================
## Catkin dependencies
## ==============================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  message_generation
)

## ==============================
## System dependencies
## ==============================
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

## ==============================
## Messages and services
## ==============================
add_message_files(
  FILES
  Viewpoint.msg
  InspectionWaypoint.msg
  InspectionWaypointArray.msg
)

add_service_files(
  FILES
  GenerateViewpoints.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

## ==============================
## Catkin package
## ==============================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scope_core
  CATKIN_DEPENDS
    roscpp
    rospy
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    tf2
    tf2_ros
    tf2_eigen
    message_runtime
  DEPENDS
    PCL
    EIGEN3
)

## ==============================
## Include directories
## ==============================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

## ==============================
## Core library
## 当前 V0.1 阶段只编译已经开始写的核心源文件
## 后续 pcd_loader、cloud_preprocessor 等模块写好后再逐步加入
## ==============================
add_library(scope_core
  src/common/parameters.cpp
  src/io/pcd_loader.cpp
  src/io/waypoint_io.cpp
  src/map/cloud_preprocessor.cpp
  src/map/normal_estimator.cpp
  src/map/target_surface.cpp
  src/viewpoint/candidate_generator.cpp
  src/viewpoint/fov_model.cpp
  src/viewpoint/coverage_evaluator.cpp
  src/safety/collision_checker.cpp
  src/safety/safety_filter.cpp
)

add_dependencies(scope_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scope_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

## ==============================
## Offline node
## ==============================
add_executable(scope_offline_node
  src/nodes/scope_offline_node.cpp
)

add_dependencies(scope_offline_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(scope_offline_node
  scope_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

## ==============================
## Install
## ==============================
install(TARGETS
  scope_core
  scope_offline_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
  config
  launch
  rviz
  sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)