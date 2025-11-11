cmake_minimum_required(VERSION 3.8)
project(orca_planner)

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

# OpenMP for parallelization (optional but recommended)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Build libraries
add_library(orca_solver
  src/orca_solver/orca_3d.cpp
)
ament_target_dependencies(orca_solver ${dependencies})
target_link_libraries(orca_solver ${EIGEN3_LIBRARIES})

add_library(spatial_hash
  src/spatial_hash/spatial_hash_3d.cpp
)
ament_target_dependencies(spatial_hash ${dependencies})
target_link_libraries(spatial_hash ${EIGEN3_LIBRARIES})

add_library(trajectory_generator
  src/trajectory_generator/orca_trajectory_generator.cpp
)
target_link_libraries(trajectory_generator
  orca_solver
  spatial_hash
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(trajectory_generator ${dependencies})

add_library(visualization_utils
  src/utils/visualization_utils.cpp
)
target_link_libraries(visualization_utils
  orca_solver
  trajectory_generator
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(visualization_utils ${dependencies})

# Build executables
add_executable(orca_planner_node
  src/orca_planner_node.cpp
)
target_link_libraries(orca_planner_node
  orca_solver
  spatial_hash
  trajectory_generator
  visualization_utils
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(orca_planner_node ${dependencies})

# ORCA Executor Node (per-drone real-time execution)
add_executable(orca_executor_node
  src/orca_executor_node.cpp
)
target_link_libraries(orca_executor_node
  orca_solver
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(orca_executor_node ${dependencies})

# Install executables
install(TARGETS
  orca_planner_node
  orca_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  orca_solver
  spatial_hash
  trajectory_generator
  visualization_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch, config, scripts
install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
