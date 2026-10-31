cmake_minimum_required(VERSION 3.8)
project(maze_mouse_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config rviz docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_maze.py scripts/validate_maze.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME maze_geometry_and_topology
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_maze.py
      --package-dir ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
