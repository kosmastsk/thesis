# Thesis

This repository contains packages, launchers and other files that are needed for my thesis.

## amcl_2d
This folder contains world files for Gazebo and the maps that were extracted from them. 
In the launch/ folder there are:
* amcl.launch: Run the amcl algorithm
* rtabmap.launch: Run RTABMAP ros, with turtlebot gazebo, rviz and teleop
* rviz_with_map.launch: Run turtlebot gazebo, rviz with a pre-loaded map, amcl and move base. It requires the [custom_navigation package](https://github.com/kosmastsk/custom-navigation) to run.

Usage:  
> catkin build amcl_2d  
> roslaunch amcl_2d rviz_with_map.launch *OR* rtabmap.launch  

## height_estimation
Contains a ROS node with a publisher and a subscriber, that reads the input from a laser altimeter sensor of the drone and the output is the height of the drone.
It calculates the average of all the laser's range values.

Usage:  
> catkin build height_estimation  
> roslaunch [hector_quadrotor_gazebo](https://github.com/kosmastsk/hector_quadrotor) italdron.launch  
> rosrun height_estimation estimator  
> rostopic echo /height

<br>
<hr>  
<b>All made using ROS Kinetic and Ubuntu 16.04</b>
