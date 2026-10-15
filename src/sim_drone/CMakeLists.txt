# Copyright 2026 Drone Waypoint Demo

cmake_minimum_required(VERSION 3.8)
project(sim_drone)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(drone_sim_core REQUIRED)

add_executable(sim_drone_node src/sim_drone_node.cpp)
target_link_libraries(sim_drone_node drone_sim_core::drone_sim_core)
ament_target_dependencies(sim_drone_node rclcpp drone_msgs)

install(TARGETS sim_drone_node DESTINATION lib/${PROJECT_NAME})

ament_package()
