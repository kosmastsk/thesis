# Drone Coverage

## Overview

This package contains an implementation of 3D coverage using a drone. Firstly, is extracts the targets within the OctoMap, according to a specific configuration of an RFID reader provided. Then, a post-process is applied to remove non-safe positions, calculate optimal orientation and minimize the cost of the total path. Finally, it navigates through the final points using smoothed obstacle-free paths, while simultaneously the drone localizes itself. It is advised to save the targets using a ROS bag file, so that the navigation can be done whenever is desired, just by playing back the ROS bag data.

## Usage

In order to extract the targets run

	roslaunch drone_coverage drone_coverage.launch
	rosbag record -O waypoints.bag /coverage/waypoints

In order to navigate through them run

	roslaunch drone_coverage coverage_navigation.launch
	rosbag play waypoints.bag

## Config file

* **params/config.yaml** Parameters related to the RFID reader, the UAV characteristics, the environment bounds, the method for reverting (x,y) points back to 6D and the subsampling step, as well as some Hill Climbing related parameters.

## Launch files

* **drone_coverage.launch:** Targets extraction

     Arguments:

     - **`octomap`** The OctoMap used for extracting targets. Default: `warehouse.ot`.

 * **online_coverage.launch:** Launching online_coverage node

 * **coverage_navigation.launch:** Navigation through the waypoints received

      Arguments:

      - **`world`** The Gazebo world used for the simulation. Default: `warehouse.world`.
      - **`map`** The OctoMap used for extracting targets. Default: `warehouse.ot`.

## Nodes

### drone_coverage

Targets extraction within an OctoMap. A subsampling is applied in the map, in order to discover the points inside the map that offer the optimal visibility. Then, these points are projected in 2D space, form a graph and Hill Climbing with Nearest Neighbor is used to approach an optimal path that consists of them. Finally, these points can be reverted back to 6D and are sent as a Trajectory ROS message to the navigation node.

#### Subscribed Topics

* **`/octomap_binary`** [octomap_msgs/Octomap]

	The map that is used for targets extraction.

* **`/projected_map`** [nav_msgs/OccupancyGrid]

	The projected OctoMap, that is used to detect non-safe points within the environment.

#### Published Topics

* **`/covered_surface`** [octomap_msgs/Octomap]

	An octomap that is created of all the points in the environment that will be within the RFID FOV at least once.

* **`/coverage/waypoints`** [trajectory_msgs/MultiDOFJointTrajectory]

	Final succession of the waypoints as a complete trajectory.

### online_coverage_node

This ROS node is responsible for publishing in real-time what the RFID reader detects and calculating the percentage of area covered.

#### Subscribed Topics

* **`/octomap_binary`** [octomap_msgs/Octomap]

	Environment's map.

* **`/amcl_pose`** [geometry_msgs/PoseStamped]

	The estimated position of the robot in real-time.

#### Published Topics

* **`/octomap_covered`** [octomap_msgs/Octomap]

	An OctoMap that contains all the points that have been inside the RFID reader FOV at least once.

* **`/octomap_covered/percentage`** [drone_gazebo/Float64Stamped]

	The percentage of the area that has been covered at any moment.

* **`/octomap_covered/volume`** [drone_gazebo/Float64Stamped]

	The volume of the area that has been covered at any moment.

### waypoint_publisher

Receives the whole trajectory that was produced from drone_coverage node and handles their serial execution. Every time the drone reaches a goal, this node receives a signal and sends the next goal to the node which is responsible for path_planning within the map. When starting, it raises the drone in 0.5 metres before sending any goal. When the drone reaches the final goal, it returns to its initial position.

#### Subscribed Topics

* **`/coverage/waypoints`** [trajectory_msgs/MultiDOFJointTrajectory]

	The whole trajectory that the drone has to follow.

* **`/goal_reached`** [std_msgs/Bool]

	Feedback received when the desired goal is reached.

#### Published Topics

* **`/next_goal`** [geometry_msgs/TransformStamped]

	The next waypoint that is sent to the path_planning module.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
