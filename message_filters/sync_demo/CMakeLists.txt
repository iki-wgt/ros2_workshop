cmake_minimum_required(VERSION 3.8)
project(sync_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_library(sync_component SHARED src/sync_component.cpp)
target_include_directories(sync_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sync_component
  rclcpp rclcpp_components geometry_msgs message_filters)

rclcpp_components_register_nodes(sync_component "SyncComponent")

install(TARGETS sync_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components geometry_msgs message_filters)
ament_package()
