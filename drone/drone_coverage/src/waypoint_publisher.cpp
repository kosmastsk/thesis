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
  _vis_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 500);
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

  ROS_INFO("[Coverage Waypoint publisher] %d coverage waypoints received\n", _number_of_waypoints);
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

  ROS_INFO("[Coverage Waypoint publisher] %zu/%d\n", _number_of_waypoints - _waypoints.size(), _number_of_waypoints);

  publish_in_rviz(next_goal);
  _goal_pub.publish(next_goal);
}

void WaypointPublisher::feedbackCallback(const std_msgs::BoolConstPtr& msg)
{
  // Is called when a goal is reached
  // Publish the next waypoint, every time a goal is reached
  // Make sure that only 'goal reached' signals are handled and waypoints still exist
  if (msg->data == 1 && !_waypoints.empty())
  {
    geometry_msgs::TransformStamped next_goal;
    next_goal.header.stamp = ros::Time::now();
    next_goal.header.frame_id = "map";

    next_goal.transform = _waypoints.front();
    _waypoints.pop();

    ROS_INFO("[Coverage Waypoint publisher] %zu/%d\n", _number_of_waypoints - _waypoints.size(), _number_of_waypoints);

    publish_in_rviz(next_goal);
    _goal_pub.publish(next_goal);
  }
}

void WaypointPublisher::publish_in_rviz(geometry_msgs::TransformStamped next_goal)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "waypoint_for_coverage";
  marker.id = _number_of_waypoints - _waypoints.size();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = next_goal.transform.translation.x;
  marker.pose.position.y = next_goal.transform.translation.y;
  marker.pose.position.z = next_goal.transform.translation.z;
  marker.pose.orientation.x = next_goal.transform.rotation.x;
  marker.pose.orientation.y = next_goal.transform.rotation.y;
  marker.pose.orientation.z = next_goal.transform.rotation.z;
  marker.pose.orientation.w = next_goal.transform.rotation.w;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 0.6;
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 1;

  _vis_pub.publish(marker);
}

}  // namespace drone_coverage

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_publisher");

  drone_coverage::WaypointPublisher pub;

  ros::spin();
  return 0;
}
