cmake_minimum_required(VERSION 3.8)
project(coug_fgo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  factor_graph_parameters
  "params/factor_graph_params.yaml"
)

get_target_property(GTSAM_TARGET_INC gtsam INTERFACE_INCLUDE_DIRECTORIES)
find_path(GTSAM_EIGEN_INCLUDE_DIR Eigen/Core
  PATHS ${GTSAM_TARGET_INC}
  PATH_SUFFIXES gtsam/3rdparty/Eigen
  NO_DEFAULT_PATH
)

add_executable(factor_graph src/factor_graph_node.cpp)
target_include_directories(factor_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GTSAM_EIGEN_INCLUDE_DIR})
ament_target_dependencies(factor_graph
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_updater
)
target_link_libraries(factor_graph
  gtsam
  gtsam_unstable
  factor_graph_parameters
)

generate_parameter_library(
  origin_manager_parameters
  "params/origin_manager_params.yaml"
)

add_executable(origin_manager src/origin_manager_node.cpp)
target_include_directories(origin_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(origin_manager
  rclcpp
  sensor_msgs
  nav_msgs
  geographic_msgs
  tf2_geometry_msgs
  geodesy
  diagnostic_updater
)
target_link_libraries(origin_manager
  origin_manager_parameters
)

install(TARGETS factor_graph origin_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

  ament_add_gtest(test_depth_factor_arm test/test_depth_factor_arm.cpp)
  target_include_directories(test_depth_factor_arm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_depth_factor_arm gtsam)

  ament_add_gtest(test_gps_factor_arm test/test_gps_factor_arm.cpp)
  target_include_directories(test_gps_factor_arm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_gps_factor_arm gtsam)

  ament_add_gtest(test_ahrs_factor test/test_ahrs_factor.cpp)
  target_include_directories(test_ahrs_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_ahrs_factor gtsam)

  ament_add_gtest(test_mag_factor_arm test/test_mag_factor_arm.cpp)
  target_include_directories(test_mag_factor_arm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_mag_factor_arm gtsam)

  ament_add_gtest(test_constant_velocity_factor test/test_constant_velocity_factor.cpp)
  target_include_directories(test_constant_velocity_factor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GTSAM_EIGEN_INCLUDE_DIR}
  )
  target_link_libraries(test_constant_velocity_factor gtsam)
endif()

ament_package()
