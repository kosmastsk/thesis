#ifndef HEIGHT_ESTIMATION_H
#define HEIGHT_ESTIMATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <vector>

namespace height_estimation
{
class Estimator
{
  private:
    ros::NodeHandle nh_;
    std_msgs::Float64 height_; // The output of the estimator
    ros::Publisher pub_;
	  ros::Subscriber sub_;

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  public:
    Estimator();
	  ~Estimator();


};  // end of class

} // end of namespace

#endif
