cmake_minimum_required(VERSION 3.0.2)
project(ed_moveit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ed
  geolib2
  geometry_msgs
  moveit_msgs
  roscpp
  shape_msgs
  std_srvs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS ed geolib2 moveit_msgs roscpp std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ed_moveit_plugin
  src/moveit_plugin.cpp
)
target_link_libraries(ed_moveit_plugin ${catkin_LIBRARIES})
add_dependencies(ed_moveit_plugin ${catkin_EXPORTED_TARGETS})
