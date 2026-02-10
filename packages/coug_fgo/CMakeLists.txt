cmake_minimum_required(VERSION 3.8)
project(coug_fgo)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(GTSAM REQUIRED)
find_package(generate_parameter_library REQUIRED)

# --- GTSAM & Eigen Setup ---
get_target_property(GTSAM_TARGET_INC gtsam INTERFACE_INCLUDE_DIRECTORIES)
find_path(GTSAM_EIGEN_INCLUDE_DIR Eigen/Core
  PATHS ${GTSAM_TARGET_INC}
  PATH_SUFFIXES gtsam/3rdparty/Eigen
  NO_DEFAULT_PATH
)

# --- Parameters ---
generate_parameter_library(
  factor_graph_parameters
  "params/factor_graph_params.yaml"
)

generate_parameter_library(
  origin_manager_parameters
  "params/origin_manager_params.yaml"
)

# --- Component Library ---
add_library(factor_graph_component SHARED
  src/factor_graph_node.cpp
  src/origin_manager_node.cpp
)

target_include_directories(factor_graph_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GTSAM_EIGEN_INCLUDE_DIR}
)

target_link_libraries(factor_graph_component
  gtsam
  gtsam_unstable
  factor_graph_parameters
  origin_manager_parameters
)

ament_target_dependencies(factor_graph_component
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  geodesy
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
)

rclcpp_components_register_nodes(factor_graph_component
  "coug_fgo::FactorGraphNode"
  "coug_fgo::OriginManagerNode"
)

# --- Executables ---
add_executable(factor_graph src/factor_graph_main.cpp)
target_link_libraries(factor_graph factor_graph_component)
ament_target_dependencies(factor_graph rclcpp rclcpp_components)

add_executable(origin_manager src/origin_manager_main.cpp)
target_link_libraries(origin_manager factor_graph_component)
ament_target_dependencies(origin_manager rclcpp rclcpp_components)

# --- Install ---
install(TARGETS
  factor_graph_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  factor_graph
  origin_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  nav_msgs
  sensor_msgs
  geometry_msgs
  geographic_msgs
  geodesy
)
ament_export_libraries(factor_graph_component)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_conversion_utils test/test_conversion_utils.cpp)
  target_include_directories(test_conversion_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_conversion_utils gtsam)
  ament_target_dependencies(test_conversion_utils geometry_msgs)

  ament_add_gtest(test_dvl_preintegrator test/test_dvl_preintegrator.cpp)
  target_include_directories(test_dvl_preintegrator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_dvl_preintegrator gtsam)

  ament_add_gtest(test_thread_safe_queue test/test_thread_safe_queue.cpp)
  target_include_directories(test_thread_safe_queue PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_thread_safe_queue sensor_msgs rclcpp)

  ament_add_gtest(test_dvl_factor test/test_dvl_factor.cpp)
  target_include_directories(test_dvl_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_dvl_factor gtsam)

  ament_add_gtest(test_dvl_preintegrated_factor test/test_dvl_preintegrated_factor.cpp)
  target_include_directories(test_dvl_preintegrated_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_dvl_preintegrated_factor gtsam)

  ament_add_gtest(test_depth_factor test/test_depth_factor.cpp)
  target_include_directories(test_depth_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_depth_factor gtsam)

  ament_add_gtest(test_gps_factor test/test_gps_factor.cpp)
  target_include_directories(test_gps_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_gps_factor gtsam)

  ament_add_gtest(test_ahrs_factor test/test_ahrs_factor.cpp)
  target_include_directories(test_ahrs_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_ahrs_factor gtsam)

  ament_add_gtest(test_mag_factor test/test_mag_factor.cpp)
  target_include_directories(test_mag_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_mag_factor gtsam)

  ament_add_gtest(test_constant_velocity_factor test/test_constant_velocity_factor.cpp)
  target_include_directories(test_constant_velocity_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_constant_velocity_factor gtsam)

  ament_add_gtest(test_auv_dynamics_factor test/test_auv_dynamics_factor.cpp)
  target_include_directories(test_auv_dynamics_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_auv_dynamics_factor gtsam)

  ament_add_gtest(test_factor_graph_node test/test_factor_graph_node.cpp)
  target_include_directories(test_factor_graph_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_factor_graph_node
    factor_graph_component
    gtsam
  )
  ament_target_dependencies(test_factor_graph_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  ament_add_gtest(test_origin_manager_node test/test_origin_manager_node.cpp)
  target_include_directories(test_origin_manager_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_origin_manager_node
    factor_graph_component
  )
  ament_target_dependencies(test_origin_manager_node
    rclcpp
    sensor_msgs
    nav_msgs
    geodesy
  )
endif()

ament_package()
