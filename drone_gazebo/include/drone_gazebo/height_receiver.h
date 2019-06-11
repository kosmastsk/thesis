#ifndef HEIGHT_RECEIVER_H
#define HEIGHT_RECEIVER_H

#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include "drone_gazebo/Float64Stamped.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

namespace height_receiver
{
class Estimator
{
private:
  ros::NodeHandle nh_;
  drone_gazebo::Float64Stamped height_;  // The output of the estimator
  ros::Publisher pub_;
  ros::Subscriber sub_;

  double _distance_from_base_link;

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

public:
  Estimator();
  ~Estimator();

};  // end of class

}  // end of namespace

#endif
