cmake_minimum_required(VERSION 3.8)
project(base_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装目录（launch、urdf、world、meshes 等）
install(DIRECTORY launch urdf world meshes config maps
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 可执行脚本到 lib/${PROJECT_NAME}
install(PROGRAMS scripts/fake_diff_drive.py scripts/arm_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
