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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
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
  ros::Subscriber _heightListener;
  ros::Subscriber _odomListener;
  ros::Publisher _odomPublisher;

  std::string _outputFrame;
  std::string _baseFrame;

  float _height;

  nav_msgs::Odometry _previousOdom;

  tf2_ros::StaticTransformBroadcaster _tfBroadcaster;

  ros::Time _lastTime;

  // Callback
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void heightCallback(const std_msgs::Float64::ConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

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
