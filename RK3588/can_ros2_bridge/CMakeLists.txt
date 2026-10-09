cmake_minimum_required(VERSION 3.8)
project(can_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_socketcan REQUIRED)

add_library(can_ros2_bridge_core
  src/socketcan_transport.cpp
  src/heartbeat_monitor.cpp
  src/rk3588_can_protocol.cpp
)
target_include_directories(can_ros2_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(can_ros2_bridge_core ros2_socketcan)

add_executable(can_receiver_node src/can_bridge_node.cpp)
target_include_directories(can_receiver_node PRIVATE include)
ament_target_dependencies(can_receiver_node rclcpp ros2_socketcan)
target_link_libraries(can_receiver_node can_ros2_bridge_core)

install(TARGETS can_ros2_bridge_core can_receiver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(can_ros2_bridge_core)
ament_export_dependencies(ros2_socketcan)
ament_package()
