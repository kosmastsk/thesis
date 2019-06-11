cmake_minimum_required(VERSION 2.8.3)
project(drone_coverage)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  drone_gazebo
  octomap_server
  octomap_ros
)

find_package(Boost REQUIRED COMPONENTS graph) # tested with Version: 1.58.0.1ubuntu1

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
  LIBRARIES drone_coverage
            coverage
            graph_utils
  CATKIN_DEPENDS roscpp
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
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
  ${OCTOMAP_LIBRARY_DIRS}
)

# Coverage library
add_library(coverage
  src/coverage.cpp
)
target_link_libraries(coverage
  ${catkin_LIBRARIES}
)

# Coverage library
add_library(graph_utils
  src/graph_utils.cpp
)
target_link_libraries(graph_utils
  ${catkin_LIBRARIES}
)

# Online Coverage library
add_library(online_coverage
  src/online_coverage.cpp
)
target_link_libraries(online_coverage
  ${catkin_LIBRARIES}
)

# Main executable
add_executable(drone_coverage
  src/drone_coverage_node.cpp)
target_link_libraries(drone_coverage
  ${catkin_LIBRARIES}
  coverage
  graph_utils
)

# Main executable
add_executable(online_coverage_node
  src/online_coverage_node.cpp)
target_link_libraries(online_coverage_node
  ${catkin_LIBRARIES}
  online_coverage
)
add_dependencies(online_coverage_node
  ${catkin_EXPORTED_TARGETS} drone_gazebo_generate_messages_cpp)

# Main executable
add_executable(waypoint_publisher
  src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher
  ${catkin_LIBRARIES}
)
