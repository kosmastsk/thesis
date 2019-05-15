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
  tf2_ros
  std_msgs
  message_generation
  tf
  topic_tools
)

add_message_files(
  FILES
  Float64Stamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
#  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs tf topic_tools
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

add_executable(estimator
  src/height_receiver_node.cpp
  src/height_receiver.cpp)
target_link_libraries(estimator
  ${catkin_LIBRARIES})
add_dependencies(estimator
  ${catkin_EXPORTED_TARGETS} drone_gazebo_generate_messages_cpp)

add_executable(message_to_tf
  src/message_to_tf.cpp)
target_link_libraries(message_to_tf
  ${catkin_LIBRARIES})
add_dependencies(message_to_tf
  ${catkin_EXPORTED_TARGETS})

add_executable(teleop src/teleop_pr2_keyboard_modificato.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
