# Drone 3D Navigation

## Overview

This package contains an implementation of drone 3D navigation in an OctoMap. The navigation is achieved using a PID controller and the [path_planning](https://github.com/kosmastsk/path_planning/tree/2faadea352245a78c9aa810d4f3a6f9f9daf7adc) package ensures an obstacle-free path inside the 3D map. By using the velocity commands, the IMU and the laser altimeter measurements, odometry estimation is calculated and is provided to the [particle_filter](https://github.com/kosmastsk/thesis/tree/master/particle_filter) package, which localizes the drone within the 3D map. Also, this package provides the ability to create trajectories for the drone to navigate through.

## Usage

Run the main node with

	roslaunch drone_3d_nav drone_3d_nav.launch


## Launch files

* **drone_3d_nav.launch:** Standard simulation

     Arguments:

     - **`world`** The Gazebo world used for the simulation. Default: `warehouse.world`.
     - **`map`** The OctoMap used for the simulation. Default: `warehouse.ot`.

## Nodes

### navigate

Implementation of a PID controller for 3D navigation.

#### Subscribed Topics

* **`/amcl_pose`** [geometry_msgs/PoseStamped]

	The output of the particle filter algorithm that provides the pose of the robot inside the map.

* **`/waypoints_smooth`** [trajectory_msgs/MultiDOFJointTrajectory]

	The waypoints that the drone must follow next.

#### Published Topics

* **`/cmd_vel`** [geometry_msgs/Twist]

	Velocity commands that will be sent to the drone for reaching the next waypoint.

* **`/cmd_vel/stamped`** [geometry_msgs/TwistStamped]

	Stamped velocity commands that will be sent to the drone for reaching the next waypoint.

* **`/goal_reached`** [std_msgs/Bool]

	Flag sent when a waypoint is reached.

### produce_odom

Estimation of drone odometry using IMU, velocity and laser height measurements.

#### Subscribed Topics

* **`/height`** [drone_gazebo/Float64Stamped]

	Laser altimeter measurement.

* **`/raw_imu`** [sensor_msgs/Imu]

	Measurements from the IMU.

* **`/cmd_vel/stamped`** [geometry_msgs/TwistStamped]

	Velocity commands that have been sent to the drone.

#### Published Topics

* **`/odom`** [nav_msgs/Odometry]

	Odometry estimation for the drone in 3D space.

### produce_trajectory

	Creation of a trajectory in 3D space that the drone can follow. This module does not ensure obstacle-free paths.

#### Published Topics

* **`/waypoints_smooth`** [trajectory_msgs/MultiDOFJointTrajectory]

	A set of points (position and orientation quaternion) that form the trajectory the drone will follow.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
