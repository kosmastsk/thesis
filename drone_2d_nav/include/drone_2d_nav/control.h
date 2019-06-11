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
  ros::Subscriber _moveBaseListener;
  ros::Publisher _cmdvelPublisher;
  float _desiredHeight;
  float _zLinear;

  // Callbacks
  void heightCallback(const std_msgs::Float64::ConstPtr& msg);
  void moveBaseCallback(const geometry_msgs::Twist::ConstPtr& msg);

public:
  Controller();
  Controller(char* argv[]);
  ~Controller();

  float getDesiredHeight()
  {
    return _desiredHeight;
  }
  void setDesiredHeight(float value)
  {
    _desiredHeight = value;
  }

  void setZLinear(float value)
  {
    _zLinear = value;
  }
  float getZLinear()
  {
    return _zLinear;
  }
};
}  // namespace control

#endif
