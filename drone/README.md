# Drone 

Drone contains the necessary packages to simulate a drone in Gazebo. Is is based on the [hector_quadrotor](http://wiki.ros.org/hector_quadrotor) package.

Before using, make sure you have downloaded the hector_quadrotor package in your workspace.

#### Usage
```sh
$ catkin build drone_gazebo
```

```sh
$ roslaunch drone_gazebo italdron.launch
```

In order to use the *teleop_twist_keyboard*, you have to run the following:
```sh
$ rosservice call /enable_motors "enable: true" 
```

> ROS Kinetic, Ubuntu 16.04
