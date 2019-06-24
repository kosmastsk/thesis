This repository contains all of the ROS packages developed for my thesis at Aristotle University of Thessaloniki.

## Abstract
Over the last years a rapid evolution of the robotics industry, and particularly that of the Unmanned Aerial Vehicles, has been observed. Despite the fact that they were primarily used for military applications, nowadays their use is constantly increasing in a variety of applications, such as the inspection of inaccessible for humans environments, events cinematography, autonomous and real-time inventorying, etc.

The use of unmanned aerial vehicles in indoor environments requires a great perception of the surrounding environment, immediate response to its changes and consequently a robust position estimation. In order to achieve these, the use of multiple available sensors is required by the drone. The navigation in an optimal way inside the environment requires the existence of an already known flight plan that will lead to the desired goal.

The present Diploma Thesis focuses on implementation of algorithms for solving the problem of fast, reliable and low-cost inventorying in the Logistics industry. The usage of drones simplifies this procedure and aims to determine every product's position with a few centimeters accuracy. This problem consists of two subproblems: a) the position estimation in the indoor environment and b) the autonomous full coverage of the area.

In order to successfully tackle the problems described above, a known 3D map in OctoMap format is used. During the research, a Particle Filter based algorithm that uses an array of distance sensors around the drone was implemented, in order to track the pose of the robot against the known map. Navigation is based on a PID position controller that ensures an obstacle free path. As for the full coverage, an extraction of the targets and then their optimal succession is performed.

Finally, a series of experiments were carried out to examine the robustness of the positioning system in three types of motion, as well as different speeds in each of these cases. At the same time, various ways of traversing the environment were examined by using different configurations of the sensor that performs the area coverage. The experiments were entirely performed in a simulated environment.

## Contents
* **drone** : ROS Metapackage
* **drone_gazebo** : Essential nodes and launchers for using the drone in Gazebo  
* **drone_description** : Drone's sensors and model description  
* **drone_2d_nav** : Autonomous navigation in a 2D map (Occupancy Grip Map)  
* **drone_3d_nav** : Autonomous navigation in a 3D map (OctoMap)  
* **drone_coverage** : Coverage path planning and navigation for 3D map  
* **particle_filter** : Localization algorithm  
* **path_planning** : Fork of a path_planning ROS module, modified for the thesis' needs  
* **experiments** : Contains scripts, worlds and maps for evaluating localization and coverage  
* **report** : LateX source of thesis report (only in Greek)  


## Dependencies
* **[ROS Kinetic](https://wiki.ros.org/kinetic)** : The whole implementation in based on ROS Kinetic and Ubuntu 16.04
* **[Gazebo 7.x](https://wiki.ros.org/gazebo)** : Simulation environment
* **[OctoMap](https://octomap.github.io)** : 3D map are represented in OctoMap format
* **[Hector Quadrotor](https://wiki.ros.org/hector_quadrotor)** : Gazebo model and several drone functionalities are used from this package
* **[teraranger_array_converter](https://github.com/kosmastsk/teraranger_array_converter)** : ROS module for using a [TeraRanger Tower](https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-tower/) sensor on top of the drone


## Installation
To build from source, clone the latest version from this repository into your catkin workspace and build the package.
```
cd ~/catkin_ws/src
git clone git@github.com:kosmastsk/thesis.git
cd ~/catkin_ws
(sudo) rosdep install drone
catkin build drone
```

## Usage
Depending on the problem, you may choose one of the available launchers that exists in the packages mentioned above.
**IMPORTANT** : After launching a file, call the following services to initialize the drone in Gazebo and the Particle Filter algorithm:
```
  rosservice call /enable_motors "enable: true"  
  rosservice call /initialize_pose  
```

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
