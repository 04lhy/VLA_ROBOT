cmake_minimum_required(VERSION 3.8)
project(base_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# world 相对引用烘焙（原因见下）：
# - gazebo_ros2_control 插件读 <parameters> 时不会按 world 文件目录解析相对路径，
#   找不到 controller_position.yaml 会抛异常、控制器全部失效；
# - mesh uri 的 "../" 相对写法在碰撞/渲染两条加载路径上的解析行为不一致，
#   实测会在 gzserver 内触发崩溃。
# 做法：git 里的 world 源码保持 ../meshes/、../config/ 相对写法（换机器无感），
# 构建时把这两处替换成 ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME} 下的绝对
# 路径再安装；改完 world 需重新 colcon build（CMAKE_CONFIGURE_DEPENDS 会自动触发）。
# ---------------------------------------------------------------------------
set(_world_share "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
set(_baked_worlds "")
foreach(_w IN ITEMS demo_scene.world test.world)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS world/${_w})
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/world/${_w}" _content)
  string(REGEX MATCHALL "\\.\\./meshes/" _mm "${_content}")
  string(REGEX MATCHALL "\\.\\./config/" _cc "${_content}")
  list(LENGTH _mm _n_mesh)
  list(LENGTH _cc _n_cfg)
  string(REPLACE "../meshes/" "file://${_world_share}/meshes/" _content "${_content}")
  string(REPLACE "../config/" "${_world_share}/config/" _content "${_content}")
  if(_content MATCHES "\\.\\./")
    message(WARNING "world/${_w} 烘焙后仍残留 '../' 引用，请检查")
  endif()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/world_gen/${_w}" "${_content}")
  list(APPEND _baked_worlds "${CMAKE_CURRENT_BINARY_DIR}/world_gen/${_w}")
  message(STATUS "baked world ${_w}: mesh=${_n_mesh}  config=${_n_cfg}")
endforeach()

# 安装目录（launch、urdf、world、meshes 等）；两个烘焙过的 world 原件排除
install(DIRECTORY launch urdf world meshes config maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "demo_scene.world" EXCLUDE
  PATTERN "test.world" EXCLUDE
)
install(FILES ${_baked_worlds}
  DESTINATION share/${PROJECT_NAME}/world
)

# 安装 Python 可执行脚本到 lib/${PROJECT_NAME}
install(PROGRAMS
  scripts/fake_diff_drive.py
  scripts/arm_control_gui.py
  scripts/tts_node.py
  scripts/vla_gazebo_bridge.py
  scripts/vla_preset_pose.py
  scripts/auto_task.py
  scripts/vla_grasp_test.py
  scripts/collect_grasp_demos.py
  scripts/collect_4room_demos.py
  scripts/simple_commander.py
  scripts/test_kdl_roundtrip.py
  scripts/nav_waypoints.py
  scripts/set_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
