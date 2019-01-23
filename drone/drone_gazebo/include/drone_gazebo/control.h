#ifndef CONTROL_HEADER
#define CONTROL_HEADER

// C++ headers
#include <iostream>
#include <cstdlib>
#include <math.h>

// ROS headers
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

namespace control
{
class Controller
{
private:
  // Variables
  ros::NodeHandle _nh;
  ros::Subscriber _heightListener;
  ros::Publisher _cmdvelPublisher;
  float _desiredHeight;

  void setHeight(int height);
  void callback(const std_msgs::Float64::ConstPtr& msg);

public:
  Controller();
  Controller(char* argv[]);
  ~Controller();

  inline int getDesiredHeight()
  {
    return _desiredHeight;
  };
  inline void setDesiredHeight(int value)
  {
    _desiredHeight = value;
  };
};
}  // namespace control

#endif
