cmake_minimum_required(VERSION 3.22)
project(qyu_ros2_simple_arm_moveit_config)

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装 launch 目录
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# 安装 config 目录
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# 安装配置文件
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 导出包
ament_package()