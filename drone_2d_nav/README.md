# Drone 2D Navigation

## Overview

This package contains an implementation of drone 2D navigation using an Occupancy Grid Map. It is based on the AMCL algorithm, configured for working with the specific UAV and the move base package. The drone receives a goal using the 2D Navigation Goal in RViz, reaches a specific height and then navigates towards the goal, while avoiding obstacles.


## Usage

Run the main node with

	roslaunch drone_2d_nav drone_2d_nav.launch

## Config files
The **/param** directory contains all the parameters required for the Local and the Global planner to properly navigate using the OGM.

## Launch files

* **drone_2d_nav.launch:** Standard simulation

     Arguments:

     - **`height`** The fixed height that the drone uses for navigation (in metres). Default: `2`.
     - **`world`** The Gazebo world used for the simulation. Default: `warehouse.world`.
     - **`map`** The world 2D map used for the simulation. Default: `warehouse.yaml`.

## Nodes

### control_node

Subscribes to the desired height and the move_base commands to provide the final velocity command to the drone.

#### Subscribed Topics

* **`/height`** [std_msgs/Float64]

	The desired height for the drone.

* **`/cmd_vel/move_base`** [geometry_msgs/Twist]

	Velocity commands received from the move_base package.


#### Published Topics

* **`/cmd_vel`** [geometry_msgs/Twist]

	Total velocity command sent to the drone.


### velocity_to_odom

Convert velocity commands to odometry message for movement in 2D.

#### Subscribed Topics

* **`/cmd_vel`** [geometry_msgs/Twist]

	Velocity commands the drone received.


#### Published Topics

* **`/odom`** [nav_msgs/Odometry]

	Odometry message that describes the drone movement.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
