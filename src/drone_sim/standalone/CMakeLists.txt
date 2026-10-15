cmake_minimum_required(VERSION 3.16)
project(drone_sim_pure_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()
find_package(GTest REQUIRED)

add_library(drone_sim_core STATIC
  ../src/kinematics.cpp
  ../src/battery_model.cpp
)
target_include_directories(drone_sim_core PUBLIC ../include)
target_compile_options(drone_sim_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(test_kinematics ../test/test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE drone_sim_core GTest::gtest_main)
add_test(NAME kinematics COMMAND test_kinematics)

add_executable(test_battery ../test/test_battery.cpp)
target_link_libraries(test_battery PRIVATE drone_sim_core GTest::gtest_main)
add_test(NAME battery COMMAND test_battery)
