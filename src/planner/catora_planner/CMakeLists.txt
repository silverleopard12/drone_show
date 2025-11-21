cmake_minimum_required(VERSION 3.8)
project(catora_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Trajectory.msg"
  "srv/GetAssignment.srv"
  "srv/GetReshapingTrajectories.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Formation Reshaper Library
add_library(catora_core SHARED
  src/formation_reshaper.cpp
  src/robot_state.cpp
  src/robot.cpp
  src/type_convertor.cpp
)

ament_target_dependencies(catora_core
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
)

# CAT-ORA Planner Node
add_executable(catora_planner_node
  src/catora_planner_node.cpp
  src/formation_reshaper.cpp
  src/robot_state.cpp
  src/robot.cpp
  src/type_convertor.cpp
)

ament_target_dependencies(catora_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  traj_utils
  Eigen3
)

# Link against generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(catora_planner_node "${cpp_typesupport_target}")

# Install
install(TARGETS
  catora_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  catora_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/catora_assignment_calculator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(catora_core)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  traj_utils
  Eigen3
)

ament_package()
