cmake_minimum_required(VERSION 3.8)
project(rs_marching_nav2_benchmark LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(ompl REQUIRED)
find_package(PNG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_library(PREFIX_OBSERVER_LIBRARY rs_benchmark_prefix_observer
  HINTS "${nav2_smac_planner_DIR}/../../../lib" REQUIRED)

add_executable(nav2_benchmark_runner runner.cpp)
ament_target_dependencies(nav2_benchmark_runner
  nav2_costmap_2d
  nav2_smac_planner
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(nav2_benchmark_runner PNG::PNG ompl::ompl ${PREFIX_OBSERVER_LIBRARY})
target_compile_options(nav2_benchmark_runner PRIVATE -Wall -Wextra -Wpedantic)

add_executable(prefix_observer_test prefix_observer_test.cpp)
ament_target_dependencies(prefix_observer_test nav2_smac_planner)
target_link_libraries(prefix_observer_test ${PREFIX_OBSERVER_LIBRARY})
enable_testing()
add_test(NAME prefix_observer_guards COMMAND prefix_observer_test)

install(TARGETS nav2_benchmark_runner DESTINATION lib/${PROJECT_NAME})
ament_package()
