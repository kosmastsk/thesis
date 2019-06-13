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
#include <drone_gazebo/height_receiver.h>

namespace height_receiver
{
Estimator::Estimator()
{
  height_.data = 0.0;
  _distance_from_base_link = 0;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Transform the height measurement from height_sensor_frame to base_link
  geometry_msgs::TransformStamped transformStamped;
  while (_distance_from_base_link == 0)
  {
    try
    {
      transformStamped = tfBuffer.lookupTransform("height_sensor_frame", "base_link", ros::Time(0));
      _distance_from_base_link = transformStamped.transform.translation.z;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }

  pub_ = nh_.advertise<drone_gazebo::Float64Stamped>("/height", 1);

  sub_ = nh_.subscribe("/lidar/height/scan", 1, &Estimator::laserCallback, this);
}

Estimator::~Estimator()
{
  ROS_INFO("[Estimator] has been destroyed");
}

void Estimator::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float height_avg = 0.0;
  int number_of_ranges = msg->ranges.size();
  float sum = 0.0;  // temp variable
  for (int i = 0; i < number_of_ranges; i++)
    sum += msg->ranges[i];

  height_avg = sum / number_of_ranges;

  height_.data = height_avg + _distance_from_base_link;
  height_.header.stamp = msg->header.stamp;
  height_.header.frame_id = "base_link";
  pub_.publish(height_);  // Publish the height in the /height topic

  ROS_INFO("Current height: [%f]", height_avg);
}
}
