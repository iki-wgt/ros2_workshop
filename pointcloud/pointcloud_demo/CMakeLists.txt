cmake_minimum_required(VERSION 3.8)
project(pointcloud_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

# library (component)
add_library(demo_component SHARED src/demo_component.cpp)

# include paths: expose our public headers + PCL
target_include_directories(demo_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# link + ament deps
target_link_libraries(demo_component ${PCL_LIBRARIES})
ament_target_dependencies(demo_component
  rclcpp rclcpp_components sensor_msgs pcl_conversions
)

# register component
rclcpp_components_register_nodes(demo_component "DemoComponent")

# install targets + headers
install(TARGETS demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# export for downstream packages
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs pcl_conversions)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
