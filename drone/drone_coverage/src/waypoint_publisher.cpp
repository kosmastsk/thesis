#include "drone_coverage/waypoint_publisher.h"

namespace drone_coverage
{
WaypointPublisher::WaypointPublisher()
{
  _goal_reached = 0;

  _waypoints_sub = _nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/coverage/waypoints", 1000,
                                                                           &WaypointPublisher::waypointCallback, this);
  _feedback_sub = _nh.subscribe<std_msgs::Bool>("/goal_reached", 1, &WaypointPublisher::feedbackCallback, this);

  _goal_pub = _nh.advertise<geometry_msgs::TransformStamped>("/next_goal", 1);
}

WaypointPublisher::~WaypointPublisher()
{
}

void WaypointPublisher::waypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  // Save all the waypoints in a stack
  _number_of_waypoints = msg->points.size();
  // Empty the queue (if not already empty) for new waypoints
  while (!_waypoints.empty())
  {
    _waypoints.pop();
  }

  ROS_INFO("%d waypoints received\n", _number_of_waypoints);
  for (unsigned int i = 0; i < _number_of_waypoints; i++)
  {
    _waypoints.push(msg->points[i].transforms[0]);
  }

  // Publish the first waypoint
  geometry_msgs::TransformStamped next_goal;
  next_goal.header.stamp = ros::Time::now();
  next_goal.header.frame_id = "map";

  next_goal.transform = _waypoints.front();
  _waypoints.pop();

  _goal_pub.publish(next_goal);
}

void WaypointPublisher::feedbackCallback(const std_msgs::BoolConstPtr& msg)
{
  // Is called when a goal is reached
  // Publish the next waypoint, every time a goal is reached
  // Make sure that only 'goal reached' signals are handled and waypoints still exist
  if (msg->data == 1 && _waypoints.empty() == false)
  {
    geometry_msgs::TransformStamped next_goal;
    next_goal.header.stamp = ros::Time::now();
    next_goal.header.frame_id = "map";

    next_goal.transform = _waypoints.front();
    _waypoints.pop();

    _goal_pub.publish(next_goal);
  }
}

}  // namespace drone_coverage

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_publisher");

  drone_coverage::WaypointPublisher pub;

  ros::spin();
  return 0;
}
