#ifndef VELOCITY_TO_ODOM_HEADER
#define VELOCITY_TO_ODOM_HEADER

// C++ headers
#include <iostream>
#include <cmath>

// ROS headers
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
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
  ros::Publisher _odomPublisher;

  nav_msgs::Odometry _previousOdom;

  ros::Time _lastTime;

  // Callback
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

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
};
}  // namespace vel_to_odom

#endif
