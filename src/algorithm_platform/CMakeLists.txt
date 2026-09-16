cmake_minimum_required(VERSION 3.8)
project(algorithm_platform)

# 1.查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 2.如果你有自定义消息 (.msg)，取消注释下面两行
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage.msg"
# )

# ===================== C++节点新增部分开始 =====================
# 编译C++可执行文件
add_executable(algws_command_test scripts/algws_command_test.cpp)
# 绑定依赖
ament_target_dependencies(
  algws_command_test
  rclcpp
  std_msgs
)
# 安装编译出来的二进制程序
install(TARGETS
  algws_command_test
  DESTINATION lib/${PROJECT_NAME}
)
# ===================== C++节点新增部分结束 =====================

# ==========================================
# 3. 自动收集 scripts/ 目录下的所有 .py 文件
# ==========================================
file(GLOB_RECURSE PYTHON_SCRIPTS "scripts/*.py")
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# 4. 安装 Launch 文件
# ==========================================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 5. 安装配置文件 (config 目录)
# ==========================================
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ==========================================
# 6. 安装 Gazebo 世界文件 (worlds 目录)
# ==========================================
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ==========================================
# 7. 安装模型文件 (models 目录)
# ==========================================
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ==========================================
# 8. 安装 RViz 配置文件 (rviz 目录)
# ==========================================
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 9. 标准打包
ament_package()
