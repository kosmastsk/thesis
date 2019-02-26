cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  tf2_ros
)

add_subdirectory(include/libPF)


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
  INCLUDE_DIRS include
  LIBRARIES particle_filter
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
 include/libPF/include
)

add_executable(particle_filter
  src/particle_filter_node.cpp
  src/particle_filter.cpp
  src/DroneMovementModel.cpp
  src/DroneState.cpp
  src/DroneObservationModel.cpp
  src/DroneStateDistribution.cpp
  src/MapModel.cpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES} PF)
add_dependencies(particle_filter ${catkin_EXPORTED_TARGETS})
