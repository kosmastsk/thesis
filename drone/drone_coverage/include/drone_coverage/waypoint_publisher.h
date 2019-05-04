#ifndef WAYPOINT_PUBLISHER_HEADER
#define WAYPOINT_PUBLISHER_HEADER

#include <iostream>
#include <queue>

#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>

namespace drone_coverage
{
class WaypointPublisher
{
private:
  ros::NodeHandle _nh;

  ros::Subscriber _waypoints_sub;
  ros::Subscriber _feedback_sub;
  ros::Publisher _goal_pub;

  bool _goal_reached;

  std::queue<geometry_msgs::Transform> _waypoints;

  int _number_of_waypoints;

  // Callbacks
  void waypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void feedbackCallback(const std_msgs::BoolConstPtr& msg);

public:
  WaypointPublisher();
  ~WaypointPublisher();
};

}  // namespace drone_coverage

#endif
