cmake_minimum_required(VERSION 2.8.3)
project(drone_3d_nav)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  drone_gazebo
  octomap_server
  roscpp
  sensor_msgs
  tf
)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_3d_nav
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_to_3d_odom
  src/velocity_to_3d_odom.cpp)
target_link_libraries(velocity_to_3d_odom
  ${catkin_LIBRARIES})
add_dependencies(velocity_to_3d_odom
  ${catkin_EXPORTED_TARGETS})

add_executable(navigate
  src/navigate.cpp)
target_link_libraries(navigate
  ${catkin_LIBRARIES})
add_dependencies(navigate
  ${catkin_EXPORTED_TARGETS})

add_executable(error_detector
  src/error_detector.cpp)
target_link_libraries(error_detector
  ${catkin_LIBRARIES})
add_dependencies(error_detector
  ${catkin_EXPORTED_TARGETS})
