#ifndef VELOCITY_TO_3D_ODOM_HEADER
#define VELOCITY_TO_3D_ODOM_HEADER

// C++ headers
#include <iostream>
#include <cmath>

// ROS headers
#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include "drone_gazebo/Float64Stamped.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/impl/utils.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace vel_to_3d_odom
{
class Converter
{
private:
  // Variables
  ros::NodeHandle _nh;

  ros::Publisher _odomPublisher;

  message_filters::Subscriber<drone_gazebo::Float64Stamped>* _height_listener;
  message_filters::Subscriber<sensor_msgs::Imu>* _imu_listener;
  message_filters::Subscriber<geometry_msgs::TwistStamped>* _velocity_listener;

  std::string _outputFrame;
  std::string _baseFrame;

  float _height;

  nav_msgs::Odometry _previousOdom;

  ros::Time _lastTime;

  void publishOdometry();

  // Callback
  void syncedCallback(const drone_gazebo::Float64StampedConstPtr& height, const sensor_msgs::ImuConstPtr& imu,
                      const geometry_msgs::TwistStampedConstPtr& velocity);

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
