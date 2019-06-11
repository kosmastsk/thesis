cmake_minimum_required(VERSION 2.8.3)
project(drone_2d_nav)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

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
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control
  src/control_node.cpp
  src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
add_dependencies(control ${catkin_EXPORTED_TARGETS})

add_executable(error_detector
  src/error_detector.cpp)
target_link_libraries(error_detector
  ${catkin_LIBRARIES})
add_dependencies(error_detector
  ${catkin_EXPORTED_TARGETS})

add_executable(vel_to_odom
  src/velocity_to_odom.cpp)
target_link_libraries(vel_to_odom ${catkin_LIBRARIES})
add_dependencies(vel_to_odom ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

#############
## Testing ##
#############
