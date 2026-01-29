cmake_minimum_required(VERSION 3.8)
project(laser_navigation_refactored)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json QUIET)
find_package(log4cplus REQUIRED)


# 如果系统没有安装 nlohmann_json，使用 FetchContent 获取
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ../../../thirdparty/
)

# ============================================================================
# 核心导航库
# ============================================================================
set(NAV_CORE_SOURCES
  src/path/straight_segment.cpp
  src/path/bezier_segment.cpp
  src/path/arc_segment.cpp
  src/planner/s_curve_planner.cpp
  src/planner/rotate.cpp
  src/controller/lqr_controller.cpp
  src/state_machine/navigation_state_machine.cpp
  src/navigation_executor.cpp
)

add_library(nav_core_lib SHARED ${NAV_CORE_SOURCES})
target_include_directories(nav_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(nav_core_lib PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  log4cplus
)
# ament_target_dependencies(nav_core_lib
#   rclcpp
#   std_msgs
#   geometry_msgs
# )
target_link_libraries(nav_core_lib PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# ============================================================================
# 示例节点
# ============================================================================
add_executable(navigation_refactored_node
  src/example/navigation_refactored_node.cpp
)
target_link_libraries(navigation_refactored_node PRIVATE
  nav_core_lib
)
# ament_target_dependencies(navigation_refactored_node
#   rclcpp
#   std_msgs
#   nav_msgs
#   geometry_msgs
#   sensor_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )
target_link_libraries(navigation_refactored_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# ============================================================================
# 安装
# ============================================================================
install(TARGETS
  nav_core_lib
  navigation_refactored_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# 导出库
ament_export_include_directories(include)
ament_export_libraries(nav_core_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
