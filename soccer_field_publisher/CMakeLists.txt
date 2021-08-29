cmake_minimum_required(VERSION 3.8)
project(soccer_field_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soccer_field_msgs REQUIRED)

# Build spl_field_publisher_node
add_executable(spl_field_publisher_node src/spl_field_publisher_node.cpp)
ament_target_dependencies(spl_field_publisher_node
  rclcpp
  soccer_field_msgs)

# Install
install(TARGETS
  spl_field_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# Install config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
