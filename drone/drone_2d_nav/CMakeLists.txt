cmake_minimum_required(VERSION 2.8.3)
project(drone_2d_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  amcl
  map_server
  move_base
  roscpp
  sensor_msgs
  tf
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_2d_nav
#  CATKIN_DEPENDS amcl map_server move_base roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############
