# Drone Gazebo

## Overview

This package contains some important utilities for using Hector Quadrotor with the Gazebo simulator. Contains an keyboard teleop for moving the drone around, handles the laser altimeter measurements and creates the necessary TF transforms from the drone movement. Also, contains the warehouse Gazebo world and the 2D/3D maps of it. Most of these utilities originally belong to the Hector Quadrotor ROS package and have been modified.

## Usage

Run the main node with

	roslaunch drone_gazebo drone_gazebo.launch


## Launch files

* **drone_gazebo.launch:** Main simulation. Launches just the drone inside a Gazebo world

     Arguments:

     - **`world`** The Gazebo world to be loaded. Default: `warehouse.world`.

 * **spawn_quadrotor.launch:** Load the necessary configuration files to use the drone in Gazebo.

      Arguments:

      - **`x`** Initial x position in the world. Default: `0`.
      - **`y`** Initial y position in the world. Default: `0`.
      - **`z`** Initial z position in the world. Default: `0`.
      - **`roll`** Initial roll orientation in the world. Default: `0`.
      - **`pitch`** Initial pitch orientation in the world. Default: `0`.
      - **`yaw`** Initial yaw orientation in the world. Default: `0`.

## Nodes

### height_receiver

Receives the measurement from the laser altimeter sensor, transforms it according to the sensor's position with regards to the quadrotor base and publishes it.

#### Subscribed Topics

* **`/lidar/height/scan`** [sensor_msgs/LaserScan]

	Raw measurement from the laser altimeter sensor.

#### Published Topics

* **`/height`** [drone_gazebo/Float64Stamped]

	The current height of the drone.

### teleop

This ROS node is responsible for receiving teleop commands from keyboard and transforming them to velocity commands.
* 'wasd' : translate
* 'zx' : altitude control
* 'qe' : yaw
* 'Shift' : run
* 'f' : stop

#### Published Topics

* **`/cmd_vel`** [geometry_msgs/Twist]

	The velocity commanded from the keyboard.

* **`/cmd_vel/stamped`** [geometry_msgs/TwistStamped]

	The velocity commanded from the keyboard, stamped.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
