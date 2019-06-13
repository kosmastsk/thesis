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
#ifndef WAYPOINT_PUBLISHER_HEADER
#define WAYPOINT_PUBLISHER_HEADER

#include <iostream>
#include <queue>

#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace drone_coverage
{
class WaypointPublisher
{
private:
  ros::NodeHandle _nh;

  ros::Subscriber _waypoints_sub;
  ros::Subscriber _feedback_sub;
  ros::Publisher _goal_pub;
  ros::Publisher _vis_pub;
  ros::Publisher _vis_array_pub;

  ros::WallTime _startTime;

  bool _goal_reached;

  std::queue<geometry_msgs::Transform> _waypoints;

  int _number_of_waypoints;

  // Callbacks
  void waypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void feedbackCallback(const std_msgs::BoolConstPtr& msg);
  void publish_in_rviz(geometry_msgs::TransformStamped next_goal);

public:
  WaypointPublisher();
  ~WaypointPublisher();
};

}  // namespace drone_coverage

#endif
