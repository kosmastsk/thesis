cmake_minimum_required(VERSION 2.8.3)
project(drone_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hector_quadrotor_model
  hector_quadrotor_controllers
  drone_description
  hector_quadrotor_actions
  hector_quadrotor_controller_gazebo
  custom_navigation_drone
)


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
  LIBRARIES drone_gazebo
  CATKIN_DEPENDS 
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

add_executable(control
  src/control_node.cpp
  src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
add_dependencies(control ${catkin_EXPORTED_TARGETS})

add_executable(vel_to_odom
  src/velocity_to_odom.cpp)
target_link_libraries(vel_to_odom ${catkin_LIBRARIES})
add_dependencies(vel_to_odom ${catkin_EXPORTED_TARGETS})
