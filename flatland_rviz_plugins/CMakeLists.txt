cmake_minimum_required(VERSION 3.5)
project(flatland_rviz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Qt6 boilerplate options from http://doc.qt.io/qt-6/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(flatland_server REQUIRED)
find_package(flatland_msgs REQUIRED)

# lua5.1
find_package(Lua 5.1 REQUIRED)

# Boost
# Boost 1.90 / CMake 4.x: Boost.System is header-only (no 'system' component) and
# date_time is unused here; only filesystem is a compiled component we need.
find_package(Boost REQUIRED COMPONENTS filesystem)

set(flatland_rviz_plugins_headers_to_moc
  include/flatland_rviz_plugins/change_rate_tool.hpp
  include/flatland_rviz_plugins/pause_tool.hpp
  include/flatland_rviz_plugins/interactive_tool.hpp
  include/flatland_rviz_plugins/spawn_model_tool.hpp
  include/flatland_rviz_plugins/change_rate_dialog.hpp
  include/flatland_rviz_plugins/load_model_dialog.hpp
  )

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/change_rate_tool.cpp
  src/change_rate_dialog.cpp
  src/pause_tool.cpp
  src/interactive_tool.cpp
  src/spawn_model_tool.cpp
  src/load_model_dialog.cpp
  ${flatland_rviz_plugins_headers_to_moc}
)

set(dependencies
  geometry_msgs
  pluginlib
  Qt6
  rclcpp
  rclcpp_lifecycle
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  std_srvs
  tf2_geometry_msgs
  flatland_server
  flatland_msgs
)

# ament_target_dependencies() was removed in ROS 2 Lyrical; link modern targets.
# Non-ament deps (Qt6, rviz_ogre_vendor) expose no ${_TARGETS} and are linked
# explicitly below.
foreach(dep IN LISTS dependencies)
  target_link_libraries(${library_name} ${${dep}_TARGETS})
endforeach()

target_include_directories(${library_name} PUBLIC
  ${Qt6Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  rviz_common::rviz_common
  Qt6::Widgets
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt6
  rviz_common
  geometry_msgs
  map_msgs
  nav_msgs
  rclcpp
)

ament_package()
