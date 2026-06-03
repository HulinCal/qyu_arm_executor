cmake_minimum_required(VERSION 3.8)
project(qyu_ros2_simple_arm)

# 设置 Python 解释器路径（避免使用 conda 的 Python）
set(Python3_EXECUTABLE "/usr/bin/python3")
set(Python_EXECUTABLE "/usr/bin/python3")

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查（除非所有源文件都添加了版权和许可证）
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint（仅在 git 仓库中工作）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装目录
install(DIRECTORY
  config
  launch
  worlds
  rviz
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# 导出包
ament_package()