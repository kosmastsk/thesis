# AMCL 2D Turtlebot

## Overview

This package contains launchers for using the Turtlebot Gazebo with AMCL algorithm.

**Keywords:** AMCL, turtlebot

The AMCL 2D Turtlebot package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone git@github.com:kosmastsk/thesis.git
	cd ../
	catkin build amcl_2d_turtlebot

## Usage

Run the main node with

	roslaunch amcl_2d_turtlebot 2d_nav.launch

## Config files

**/param** Contains the parameter files from the move base, using the Teb Local Planner and Global Planner

## Launch files

* **2d_nav.launch:** Standard simulation with turtlebot in Gazebo, using move base and AMCL algorithm

     - **`world`** Path of the world file to be used. Default: `$(find drone_gazebo)/worlds/grannie.world`.

* **rtabmap.launch:** SLAM using RTABMAP algorithm

     - **`rviz`** Set to true if you want to use rviz instead of the default RTABMAP viz. Default: `false`.
     - **`world`** Path of the world file to be used. Default: `$(find drone_gazebo)/worlds/grannie.world`.

* **gmapping.launch:** SLAM using gmapping algorithm and the move with AMCL and move base

     - **`world`** Path of the world file to be used. Default: `$(find drone_gazebo)/worlds/grannie.world`.     
