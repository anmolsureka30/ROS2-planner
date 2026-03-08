cmake_minimum_required(VERSION 3.8)
project(hybrid_astar_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Find OMPL
find_package(ompl REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

# Source files
set(SOURCES
  src/hybrid_astar_planner.cpp
  src/search_engine.cpp
  src/motion_model.cpp
  src/collision_checker.cpp
  src/heuristic_calculator.cpp
  src/analytic_expander.cpp
  src/cost_function.cpp
  src/path_smoother.cpp
)

# Shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OMPL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES hybrid_astar_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_core hybrid_astar_planner_plugin.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_motion_model test/test_motion_model.cpp)
  target_link_libraries(test_motion_model ${PROJECT_NAME} ${OMPL_LIBRARIES})
  ament_target_dependencies(test_motion_model ${DEPENDENCIES})

  ament_add_gtest(test_cost_function test/test_cost_function.cpp)
  target_link_libraries(test_cost_function ${PROJECT_NAME} ${OMPL_LIBRARIES})
  ament_target_dependencies(test_cost_function ${DEPENDENCIES})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()
