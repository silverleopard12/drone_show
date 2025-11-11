cmake_minimum_required(VERSION 3.8)
project(dl_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(traj_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Dependencies list
set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  traj_utils
  quadrotor_msgs
  plan_env
)

# Build neural network library
add_library(neural_collision_net
  src/neural_network/neural_collision_net.cpp
)
target_include_directories(neural_collision_net PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(neural_collision_net
  ${EIGEN3_LIBRARIES}
)

# Build collision checker library
add_library(swarm_collision_checker
  src/collision_checker/swarm_collision_checker.cpp
)
target_include_directories(swarm_collision_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(swarm_collision_checker
  ${EIGEN3_LIBRARIES}
)

# Build trajectory generator library
add_library(offline_trajectory_generator
  src/trajectory_generator/offline_trajectory_generator.cpp
)
target_include_directories(offline_trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(offline_trajectory_generator
  neural_collision_net
  swarm_collision_checker
  ${EIGEN3_LIBRARIES}
)

# Build DL planner node executable
add_executable(dl_planner_node
  src/dl_planner_node.cpp
)
target_include_directories(dl_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dl_planner_node
  neural_collision_net
  swarm_collision_checker
  offline_trajectory_generator
)
ament_target_dependencies(dl_planner_node ${dependencies})

# Install libraries
install(TARGETS
  neural_collision_net
  swarm_collision_checker
  offline_trajectory_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS
  dl_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch config models scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
