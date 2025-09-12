cmake_minimum_required(VERSION 3.16)
project(room_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME})

ament_package()
