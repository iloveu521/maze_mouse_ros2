cmake_minimum_required(VERSION 3.22)
project(maze_mouse_navigation LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(maze_navigation_core src/core.cpp)
target_include_directories(maze_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(maze_navigation_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(maze_navigator src/navigator.cpp)
target_link_libraries(maze_navigator maze_navigation_core)
ament_target_dependencies(maze_navigator rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)
target_compile_options(maze_navigator PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS maze_navigation_core maze_navigator
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config docs rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(core_tests test/core_tests.cpp)
  target_link_libraries(core_tests maze_navigation_core)
endif()
ament_export_include_directories(include)
ament_export_libraries(maze_navigation_core)
ament_package()
