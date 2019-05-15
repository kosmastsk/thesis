#ifndef HEIGHT_RECEIVER_H
#define HEIGHT_RECEIVER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "drone_gazebo/Float64Stamped.h"
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <vector>

namespace height_receiver
{
class Estimator
{
private:
  ros::NodeHandle nh_;
  drone_gazebo::Float64Stamped height_;  // The output of the estimator
  ros::Publisher pub_;
  ros::Subscriber sub_;

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

public:
  Estimator();
  ~Estimator();

};  // end of class

}  // end of namespace

#endif
