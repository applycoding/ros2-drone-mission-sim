cmake_minimum_required(VERSION 3.8)
project(drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure dynamics. No ROS headers, so the same sources build under standalone/.
add_library(drone_sim_core STATIC
  src/kinematics.cpp
  src/battery_model.cpp
)
target_include_directories(drone_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(drone_sim_core PUBLIC cxx_std_17)

add_executable(drone_sim_node src/drone_sim_node.cpp)
target_link_libraries(drone_sim_node drone_sim_core)
ament_target_dependencies(drone_sim_node
  geometry_msgs
  interfaces
  nav_msgs
  rclcpp
  tf2_ros
)

install(TARGETS drone_sim_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics drone_sim_core)
  ament_add_gtest(test_battery test/test_battery.cpp)
  target_link_libraries(test_battery drone_sim_core)
endif()

ament_package()
