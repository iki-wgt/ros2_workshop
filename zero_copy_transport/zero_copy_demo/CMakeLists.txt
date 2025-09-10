cmake_minimum_required(VERSION 3.8)
project(zero_copy_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Build the shared library that contains both components
add_library(zero_copy_components SHARED src/zero_copy.cpp)
ament_target_dependencies(zero_copy_components
  rclcpp rclcpp_components std_msgs)

# Register both components
rclcpp_components_register_nodes(zero_copy_components
  "TalkerComponent"
  "ListenerComponent")

# Install the library
install(TARGETS zero_copy_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export deps for downstream users
ament_export_dependencies(rclcpp rclcpp_components std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
