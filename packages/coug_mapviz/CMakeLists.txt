cmake_minimum_required(VERSION 3.8)
project(coug_mapviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Skip builds on incompatible architectures
if(NOT "$ENV{ROS_BUILD_ARCH}" STREQUAL "amd64")
  message(STATUS "Skipping coug_mapviz build on $ENV{ROS_BUILD_ARCH}")
  find_package(ament_cmake REQUIRED)
  ament_package()
  return()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(mapviz REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui OpenGL Widgets)
find_package(OpenGL REQUIRED)

add_definitions(-DWFlags=WindowFlags)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(UI_FILES ui/coug_waypoints_config.ui)
set(SRC_FILES src/coug_waypoints_plugin.cpp src/coug_waypoint_manager.cpp)
set(HEADER_FILES include/${PROJECT_NAME}/coug_waypoints_plugin.hpp)

qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME} SHARED
  ${MOC_FILES}
  ${SRC_FILES}
  ${UI_SRC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PACKAGE_NAME}>)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  mapviz
  pluginlib
  rclcpp
  swri_transform_util
  tf2
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
  OpenGL::GL
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

install(DIRECTORY launch mapviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  mapviz
  Qt5
  rclcpp
  swri_transform_util
  tf2
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

endif()

ament_package()
