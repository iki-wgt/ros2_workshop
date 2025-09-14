cmake_minimum_required(VERSION 3.8)
project(pointcloud_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters segmentation kdtree common)

add_library(segmentation_component SHARED
  src/segmentation_component.cpp
)

target_include_directories(segmentation_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(segmentation_component
  ${PCL_LIBRARIES}
)

ament_target_dependencies(segmentation_component
  rclcpp
  rclcpp_components
  sensor_msgs
  message_filters
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

rclcpp_components_register_nodes(segmentation_component "pointcloud_segmentation::SegmentationComponent")

# install targets + headers
install(TARGETS segmentation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# install runtime assets (so ros2 launch can find them)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs message_filters tf2_ros tf2_sensor_msgs pcl_conversions
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
