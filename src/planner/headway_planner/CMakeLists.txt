cmake_minimum_required(VERSION 3.8)
project(headway_planner)

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
find_package(Eigen3 REQUIRED)

# Include nlohmann/json (system package)
# On Ubuntu: sudo apt install nlohmann-json3-dev
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  message(WARNING "nlohmann_json not found. Attempting to use system headers.")
  # If not found as package, try to include system headers
  include_directories(/usr/include/nlohmann)
endif()

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
)

#=============================================================================
# Source files
#=============================================================================

# Utilities
set(UTILS_SOURCES
  src/utils/geometry_utils.cpp
)

# Collision detection
set(COLLISION_SOURCES
  src/collision_detector/event_extractor.cpp
)

# Scheduler
set(SCHEDULER_SOURCES
  src/scheduler/headway_scheduler.cpp
  src/scheduler/greedy_scheduler.cpp
  src/scheduler/milp_scheduler.cpp
)

# Altitude slotting
set(ALTITUDE_SOURCES
  src/altitude/slot_allocator.cpp
)

# Trajectory generation
set(TRAJECTORY_SOURCES
  src/trajectory/plan_generator.cpp
)

# Main node
set(NODE_SOURCES
  src/headway_planner_node.cpp
)

#=============================================================================
# Executable: headway_planner_node
#=============================================================================

add_executable(headway_planner_node
  ${NODE_SOURCES}
  ${UTILS_SOURCES}
  ${COLLISION_SOURCES}
  ${SCHEDULER_SOURCES}
  ${ALTITUDE_SOURCES}
  ${TRAJECTORY_SOURCES}
)

ament_target_dependencies(headway_planner_node
  ${dependencies}
)

target_link_libraries(headway_planner_node
  ${EIGEN3_LIBRARIES}
)

if(nlohmann_json_FOUND)
  target_link_libraries(headway_planner_node nlohmann_json::nlohmann_json)
endif()

#=============================================================================
# Executable: headway_executor_node
#=============================================================================

add_executable(headway_executor_node
  src/headway_executor_node.cpp
)

ament_target_dependencies(headway_executor_node
  ${dependencies}
)

target_link_libraries(headway_executor_node
  ${EIGEN3_LIBRARIES}
)

#=============================================================================
# Install
#=============================================================================

# Install executables
install(TARGETS
  headway_planner_node
  headway_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
