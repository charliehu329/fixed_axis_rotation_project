cmake_minimum_required(VERSION 3.8)

project(fixed_axis_rotation_interfaces)

# ============================================================
# 编译选项
# ============================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ============================================================
# 查找依赖
# ============================================================

find_package(
  ament_cmake
  REQUIRED
)

find_package(
  rosidl_default_generators
  REQUIRED
)

# ============================================================
# 生成自定义 ROS 2 消息
# ============================================================

rosidl_generate_interfaces(
  ${PROJECT_NAME}

  "msg/FixedAxisCommand.msg"
)

# ============================================================
# 导出运行时依赖
# ============================================================

ament_export_dependencies(
  rosidl_default_runtime
)

# ============================================================
# 完成 ROS 2 包配置
# ============================================================

ament_package()