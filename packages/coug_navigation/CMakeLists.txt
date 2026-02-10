cmake_minimum_required(VERSION 3.8)
project(coug_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)

# --- Parameters ---
generate_parameter_library(
  waypoint_follower_parameters
  "params/waypoint_follower_params.yaml"
)

# --- Component Library ---
add_library(waypoint_follower_component SHARED src/waypoint_follower_node.cpp)
target_include_directories(waypoint_follower_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_follower_component
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  std_msgs
  diagnostic_updater
)
target_link_libraries(waypoint_follower_component
  waypoint_follower_parameters
)

rclcpp_components_register_nodes(waypoint_follower_component
  "coug_navigation::WaypointFollowerNode"
)

# --- Executables ---
add_executable(waypoint_follower src/waypoint_follower_main.cpp)
target_link_libraries(waypoint_follower
  waypoint_follower_component
)
ament_target_dependencies(waypoint_follower rclcpp rclcpp_components)

# --- Install ---
install(TARGETS
  waypoint_follower_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS waypoint_follower
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_follower_node test/test_waypoint_follower_node.cpp)
  target_include_directories(test_waypoint_follower_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_waypoint_follower_node
    waypoint_follower_component
  )
  ament_target_dependencies(test_waypoint_follower_node
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
  )
endif()

ament_package()
