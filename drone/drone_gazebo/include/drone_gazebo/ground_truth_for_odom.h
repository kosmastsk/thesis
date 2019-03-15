#ifndef GROUND_TRUTH_FOR_ODOM_HEADER
#define GROUND_TRUTH_FOR_ODOM_HEADER

// C++ headers
#include <iostream>

// ROS heas
#include <ros/ros.h>

#include "nav_msgs/Odometry.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gnd_for_odom
{
class Gnd
{
private:
  // Variables
  ros::NodeHandle _nh;
  ros::Subscriber _gndListener;
  ros::Publisher _odomPublisher;

  std::string _outputFrame;
  std::string _baseFrame;

  tf2_ros::TransformBroadcaster _tfBroadcaster;

  nav_msgs::Odometry _previousOdom;

  // Callback
  void gndCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  Gnd();
  Gnd(char* argv[]);
  ~Gnd();
};
}  // namespace vel_to_odom

#endif
