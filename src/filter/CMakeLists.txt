cmake_minimum_required(VERSION 3.5)
project(filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(filter SHARED src/filter_node.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(filter
  rclcpp
  sensor_msgs
  rclcpp_components
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# 注册组件并生成名为 filter_node 的可执行文件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN PointCloudProcessor
  EXECUTABLE filter_node
)

install(TARGETS filter DESTINATION lib)

ament_export_dependencies(rclcpp_components)
ament_package()