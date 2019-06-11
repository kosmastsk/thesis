cmake_minimum_required(VERSION 2.8.3)
project(drone_3d_nav)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  octomap_server
  roscpp
  sensor_msgs
  tf
  trajectory_msgs
  genmsg
  drone_gazebo
  dynamic_reconfigure
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/pid.cfg
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
#  INCLUDE_DIRS
#  LIBRARIES
#  CATKIN_DEPENDS
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

add_executable(produce_odom
  src/produce_odom.cpp)
target_link_libraries(produce_odom
  ${catkin_LIBRARIES})
add_dependencies(produce_odom
  ${catkin_EXPORTED_TARGETS})

add_executable(navigate
  src/navigate.cpp)
target_link_libraries(navigate
  ${catkin_LIBRARIES})
add_dependencies(navigate
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

add_executable(produce_trajectory
  src/produce_trajectory.cpp)
target_link_libraries(produce_trajectory
  ${catkin_LIBRARIES})
add_dependencies(produce_trajectory
  ${catkin_EXPORTED_TARGETS})
