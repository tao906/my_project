cmake_minimum_required(VERSION 3.8)
project(fishbot_dlite_planner)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# 2. 创建库
add_library(fishbot_dlite_planner SHARED src/dlite_planner.cpp)
target_include_directories(fishbot_dlite_planner PUBLIC include)
ament_target_dependencies(fishbot_dlite_planner
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
)

# 3. 注册插件
pluginlib_export_plugin_description_file(nav2_core nav2_dlite_planner_plugin.xml)

# 4. 安装
install(TARGETS fishbot_dlite_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES nav2_dlite_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()