cmake_minimum_required(VERSION 3.8)
project(flex_core)

# ============================================================================
# 1. 编译选项配置
# ============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# GCC 特定选项
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-class-memaccess)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# 2. 查找 ROS2 依赖包
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flex_msgs REQUIRED)

# ============================================================================
# 3. 查找第三方库依赖
# ============================================================================
# Eigen3 线性代数库
find_package(Eigen3 REQUIRED)

# YAML-CPP 配置文件解析库
find_package(yaml-cpp REQUIRED)

# Qt5 库（用于串口通信和 GUI）
find_package(Qt5 COMPONENTS Widgets Core SerialPort REQUIRED)

# 线程库
find_package(Threads REQUIRED)

# ============================================================================
# 4. 创建共享库
# ============================================================================
# Qt5 MOC 处理（必须在创建库之前，因为 RemoteControlParser 使用了 Q_OBJECT）
qt5_wrap_cpp(remote_parser_moc_src
  include/flex_core/RemoteControlParser.hpp
)

# 注意：共享库只包含实现文件，不包含包含 main() 函数的节点文件
add_library(${PROJECT_NAME} SHARED
  src/FlexCore.cpp
  src/MFAC.cpp
  src/RemoteControlParser.cpp
  ${remote_parser_moc_src}  # MOC 生成的文件必须添加到库中
)

# 设置库的头文件包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# 链接 ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  yaml-cpp
  flex_msgs
)

# 链接系统库和第三方库
target_link_libraries(${PROJECT_NAME}
  Threads::Threads
  Qt5::Widgets
  Qt5::Core
  Qt5::SerialPort
  yaml-cpp
)

# ============================================================================
# 5. 创建可执行文件
# ============================================================================

# FlexCore 控制节点（主控制节点）
# 注意：实现文件已在共享库中，这里只需要节点入口文件
add_executable(flex_control_core_node
  node/FlexCoreNode.cpp
)

ament_target_dependencies(flex_control_core_node
  rclcpp
  yaml-cpp
  flex_msgs
)

target_link_libraries(flex_control_core_node
  ${PROJECT_NAME}
)

# RemoteControl 节点（遥控器解析节点）
# 注意：MOC 文件已在共享库中处理，这里不需要重复处理
# 注意：实现文件已在共享库中，这里只需要节点入口文件
add_executable(remote_control_core_node
  node/RemoteCoreNode.cpp
)

ament_target_dependencies(remote_control_core_node
  rclcpp
  yaml-cpp
  flex_msgs
)

target_link_libraries(remote_control_core_node
  ${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::SerialPort
)

# ============================================================================
# 6. 安装规则
# ============================================================================

# 安装共享库
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
install(TARGETS
  flex_control_core_node
  remote_control_core_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/driver_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装参数文件
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

# 安装 Launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# 安装文档文件（可选）
install(FILES
  README_PYTHON.md
  QUICK_START.md
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 导出目标供其他包使用
install(EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ============================================================================
# 7. 测试支持（可选）
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================================
# 8. 完成 ament 包配置
# ============================================================================
ament_package()
