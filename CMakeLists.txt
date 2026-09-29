cmake_minimum_required(VERSION 3.8)
project(ed_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ed REQUIRED)
find_package(geolib2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(ed_moveit_plugin SHARED
  src/moveit_plugin.cpp
)
target_include_directories(ed_moveit_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ed_moveit_plugin PUBLIC
  ed
  geolib2::geolib
  geometry_msgs::geometry_msgs
  moveit_msgs::moveit_msgs
  pluginlib::pluginlib
  rclcpp::rclcpp
  shape_msgs::shape_msgs
  std_srvs::std_srvs
)

pluginlib_export_plugin_description_file(ed plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ed_moveit_plugin
  EXPORT export_ed_moveit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_ed_moveit HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ed
  geolib2
  geometry_msgs
  moveit_msgs
  pluginlib
  rclcpp
  shape_msgs
  std_srvs
)

ament_package()
