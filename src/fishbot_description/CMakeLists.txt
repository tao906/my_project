cmake_minimum_required(VERSION 3.8)
project(fishbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 添加这些依赖查找，确保编译时检查环境
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# ==================== 关键安装规则 ====================
# 1. 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. 安装 urdf 文件夹 (包含 xacro)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# 3. 安装 config 文件夹 (包含 controller.yaml)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# === 新增：安装 world 文件夹 ===
install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}
)

# === 新增：安装 maps 文件夹 ===
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# ====================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()