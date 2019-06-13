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
#ifndef HEIGHT_RECEIVER_H
#define HEIGHT_RECEIVER_H

#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include "drone_gazebo/Float64Stamped.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

namespace height_receiver
{
class Estimator
{
private:
  ros::NodeHandle nh_;
  drone_gazebo::Float64Stamped height_;  // The output of the estimator
  ros::Publisher pub_;
  ros::Subscriber sub_;

  double _distance_from_base_link;

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

public:
  Estimator();
  ~Estimator();

};  // end of class

}  // end of namespace

#endif
