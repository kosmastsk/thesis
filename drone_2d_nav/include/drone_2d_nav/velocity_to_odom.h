#ifndef VELOCITY_TO_ODOM_HEADER
#define VELOCITY_TO_ODOM_HEADER

// C++ headers
#include <iostream>
#include <cmath>

// ROS headers
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"

#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/impl/utils.h>

namespace vel_to_odom
{
class Converter
{
private:
  // Variables
  ros::NodeHandle _nh;
  ros::Subscriber _cmdVelListener;
  ros::Publisher _odomPublisher;

  std::string _outputFrame;
  std::string _baseFrame;

  nav_msgs::Odometry _previousOdom;

  ros::Time _lastTime;

  void publishOdometry();

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
  void updateOdomTime(ros::Time t)
  {
    _previousOdom.header.stamp = t;
  }
};
}  // namespace vel_to_odom

#endif
