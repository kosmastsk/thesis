#ifndef VELOCITY_TO_ODOM_HEADER
#define VELOCITY_TO_ODOM_HEADER

// C++ headers
#include <iostream>
#include <cmath>

// ROS headers
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace vel_to_odom
{
class Converter
{
private:
  // Variables
  ros::NodeHandle _nh;
  ros::Subscriber _cmdVelListener;
  ros::Subscriber _heightListener;
  ros::Publisher _odomPublisher;

  float _height;

  nav_msgs::Odometry _previousOdom;

  ros::Time _lastTime;

  // Callback
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void heightCallback(const std_msgs::Float64::ConstPtr& msg);

public:
  Converter();
  Converter(char* argv[]);
  ~Converter();

  nav_msgs::Odometry getPreviousOdom()
  {
    return _previousOdom;
  }
  void setPreviousOdom(nav_msgs::Odometry odometry)
  {
    _previousOdom = odometry;
  }

  float getHeight()
  {
    return _height;
  }
  void setHeight(float value)
  {
    _height = value;
  }
};
}  // namespace vel_to_odom

#endif
