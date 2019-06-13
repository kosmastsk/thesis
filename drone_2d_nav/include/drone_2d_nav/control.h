/*
* Copyright (c) 2019 Kosmas Tsiakas
*
* GNU GENERAL PUBLIC LICENSE
*    Version 3, 29 June 2007
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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
#include "drone_gazebo/Float64Stamped.h"

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
  void heightCallback(const drone_gazebo::Float64Stamped::ConstPtr& msg);
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
