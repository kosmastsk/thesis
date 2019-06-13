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
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl_pose,
              const nav_msgs::OdometryConstPtr& gnd_state)
{
  double error_x, error_y;
  error_x = amcl_pose->pose.pose.position.x - gnd_state->pose.pose.position.x;
  error_y = amcl_pose->pose.pose.position.y - gnd_state->pose.pose.position.y;

  ROS_INFO_ONCE("[/amcl_pose] - [/ground_truth/state]\n");
  ROS_INFO("Error in [x, y] : [%f, %f]\n", error_x, error_y);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "error_detector");

  ros::NodeHandle nh;

  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_sub(nh, "/amcl_pose", 1);
  message_filters::Subscriber<nav_msgs::Odometry> gnd_sub(nh, "/ground_truth/state", 1);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amcl_sub, gnd_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
