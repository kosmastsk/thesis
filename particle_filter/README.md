# Particle Filter

## Overview

This package contains an implementation of a 3D localization algorithm that is based on a Particle Filter. It consists of a *Movement model* that is acting according to the odom transforms, an *Observation model* that is based on the laser measurements, a *Map model* that contains important OctoMap functionalities and a *State Distribution model* that is responsible for the distribution of particles around the map.

## Usage

Run the main node with

	roslaunch particle_filter particle_filter.launch


## Config file

* **params/config.yaml** Parameters related to the MCL algorithm.

## Launch files

* **particle_filter.launch:** Execution of the algorithm


## Nodes

### particle_filter

Initialize particles with the same weight around a known initial position with a Gaussian Distribution. Then, according to the Movement model the particles move around the map and using the Observation model their weights are updated. The movement model is based on the TF transforms. When the number of effective particles is less than the total number of particles, a resampling is performed. A total pose estimation is extracted from the mean of 100% of the particles.

#### Subscribed Topics

* **`/scan`** [sensor_msgs/LaserScan]

	The laser measurements around the drone.

* **`/amcl/initial_pose`** [geometry_msgs/PoseWithCovarianceStamped]

	The initial pose of the drone in the map.

* **`/ground_truth/state`** [nav_msgs/Odometry]

	The real position of the drone in the world.

#### Published Topics

* **`/amcl_pose`** [geometry_msgs/PoseStamped]

	Mean state of all particles.

* **`/amcl/particlecloud`** [geometry_msgs/PoseArray]

	The pose estimation of each particle.

* **`/amcl/initial_pose`** [geometry_msgs/PoseWithCovarianceStamped]

	The initial pose of the drone in the map.

#### Services

* **`initialize_pose`** ([std_srvs/Empty])

	Initializes the particle filter algorithm and distributes the particles around the known initial position with a Gaussian Distribution.

		rosservice call /initialize_pose

* **`global_localization`** ([std_srvs/Empty])

	Initializes the particle filter algorithm and distributes the particles around the map with a Uniform Distribution.

		rosservice call /global_localization

* **`repair_pose`** ([std_srvs/Empty])

	By using the information of the ground_truth topic, a pose repair is possible if needed. Created just for simulation and testing purposes.

		rosservice call /repair_pose

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
