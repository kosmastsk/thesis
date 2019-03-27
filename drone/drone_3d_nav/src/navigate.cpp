#include "drone_3d_nav/navigate.h"

namespace navigate
{
Navigator::Navigator()
{
  // Parameters

  _nh.param<double>("/kp", _kp, 0.5);
  _nh.param<double>("/ki", _ki, 0.0002);
  _nh.param<double>("/kd", _kd, 0.00005);
  _nh.param<double>("/kp_yaw", _kp_yaw, 0.5);
  _nh.param<double>("/ki_yaw", _ki_yaw, 0.0002);
  _nh.param<double>("/kd_yaw", _kd_yaw, 0.00005);
  _nh.param<float>("/tolerance", _tolerance, 0.1);

  _must_exit = false;
  _waypoint_number = 0;
  _waypoints_received = false;
  _dt = 0;

  _error_x = 0;
  _error_y = 0;
  _error_z = 0;
  // _error_roll = 0;
  // _error_pitch = 0;
  _error_yaw = 0;

  _prev_error_x = 0;
  _prev_error_y = 0;
  _prev_error_z = 0;
  // _prev_error_roll = 0;
  // _prev_error_pitch = 0;
  _prev_error_yaw = 0;

  _rise = 1;

  _proportional_x = 0;
  _proportional_y = 0;
  _proportional_z = 0;
  // _proportional_roll = 0;
  // _proportional_pitch = 0;
  _proportional_yaw = 0;

  _integral_x = 0;
  _integral_y = 0;
  _integral_z = 0;
  // _integral_roll = 0;
  // _integral_pitch = 0;
  _integral_yaw = 0;

  _derivative_x = 0;
  _derivative_y = 0;
  _derivative_z = 0;
  // _derivative_roll = 0;
  // _derivative_pitch = 0;
  _derivative_yaw = 0;

  _action_x = 0;
  _action_y = 0;
  _action_z = 0;
  // _action_roll = 0;
  // _action_pitch = 0;
  _action_yaw = 0;

  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  _odom_sub = _nh.subscribe("/ground_truth/state", 1, &Navigator::odomCallback, this);
  _waypoints_sub = _nh.subscribe("/waypoints_smooth", 1, &Navigator::waypointCallback, this);
}

Navigator::~Navigator()
{
}

void Navigator::waypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  // The first item in the points array is the initial position
  _waypoint_number = 1;
  _number_of_waypoints = msg->points.size();
  ROS_INFO("%d waypoints received\n", _number_of_waypoints);
  for (unsigned int i = 0; i < _number_of_waypoints; i++)
  {
    _waypoints.push(msg->points[i].transforms[0]);
  }
  _current_goal = _waypoints.front();
  _waypoints.pop();  // Remove this element from the queue
  _waypoints_received = true;
}

void Navigator::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // If the waypoints have not received, we cannot proceed with the navigation.
  if (!_waypoints_received)
  {
    ROS_WARN_ONCE("Waypoints not received. Skipping odometry...\n");
    return;
  }

  _pose.translation.x = msg->pose.pose.position.x;
  _pose.translation.y = msg->pose.pose.position.y;
  _pose.translation.z = msg->pose.pose.position.z;

  _pose.rotation = msg->pose.pose.orientation;  // geometry_msgs::Quaternion showing the current orientation

  // Get the Yaw from current goal and odometry measurement
  tf2::Quaternion current_goal_quat, pose_quat;
  tf2::fromMsg(_current_goal.rotation, current_goal_quat);
  tf2::fromMsg(_pose.rotation, pose_quat);

  double current_goal_yaw = tf2::impl::getYaw(current_goal_quat);
  double pose_yaw = tf2::impl::getYaw(pose_quat);

  _dt = ros::Time::now().toSec() - _dt;

  // Find errors and PID values to calculate next action
  _error_x = _current_goal.translation.x - _pose.translation.x;
  _error_y = _current_goal.translation.y - _pose.translation.y;
  _error_z = _current_goal.translation.z - _pose.translation.z;
  // _error_roll = current_goal_roll - pose_roll;     // _current_goal.rotation.x - _pose.rotation.x;
  // _error_pitch = current_goal_pitch - pose_pitch;  // _current_goal.rotation.y - _pose.rotation.y;
  _error_yaw = current_goal_yaw - pose_yaw;  // _current_goal.rotation.z - _pose.rotation.z;

  _proportional_x = _kp * _error_x;
  _proportional_y = _kp * _error_y;
  _proportional_z = _kp * _error_z;
  // _proportional_roll = _kp * _error_roll;
  // _proportional_pitch = _kp * _error_pitch;
  _proportional_yaw = _kp_yaw * _error_yaw;

  _integral_x = _ki * (_integral_x + _error_x * _dt);
  _integral_y = _ki * (_integral_y + _error_y * _dt);
  _integral_z = _ki * (_integral_z + _error_z * _dt);
  // _integral_roll += _ki * _error_roll;
  // _integral_pitch += _ki * _error_pitch;
  _integral_yaw = _ki_yaw * (_integral_yaw + _error_yaw * _dt);

  _derivative_x = _kd * (_error_x - _prev_error_x);
  _derivative_y = _kd * (_error_y - _prev_error_y);
  _derivative_z = _kd * (_error_z - _prev_error_z);
  // _derivative_roll = _kd * (_error_roll - _prev_error_roll);
  // _derivative_pitch = _kd * (_error_pitch - _prev_error_pitch);
  _derivative_yaw = _kd_yaw * (_error_yaw - _prev_error_yaw);

  _prev_error_x = _error_x;
  _prev_error_y = _error_y;
  _prev_error_z = _error_z;
  // _prev_error_roll = _error_roll;
  // _prev_error_pitch = _error_pitch;
  _prev_error_yaw = _error_yaw;

  _action_x = _proportional_x + _integral_x + _derivative_x;
  _action_y = _proportional_y + _integral_y + _derivative_y;
  _action_z = 5 * _proportional_z + _integral_z + _derivative_z;
  // _action_roll = _proportional_roll + _integral_roll + _derivative_roll;
  // _action_pitch = _proportional_pitch + _integral_pitch + _derivative_pitch;
  _action_yaw = _proportional_yaw + _integral_yaw + _derivative_yaw;

  _twist.linear.x = _action_x;
  _twist.linear.y = _action_y;
  _twist.linear.z = _action_z;
  _twist.angular.x = 0;  //_action_roll;
  _twist.angular.y = 0;  //_action_pitch;
  _twist.angular.z = _action_yaw;

  ROS_INFO("Error (X, Y, Z, Yaw) : (%0.2f, %0.2f, %0.2f, %0.2f) \n", _error_x, _error_y, _error_z, _error_yaw);

  ROS_INFO("Action (X, Y, Z, Yaw) : (%0.2f, %0.2f, %0.2f, %0.2f) \n", _action_x, _action_y, _action_z, _action_yaw);

  // Ensure that the drone's position is in accepted range error
  if ((fabs(_error_x) <= _tolerance) && (fabs(_error_y) <= _tolerance) && (fabs(_error_z) <= _tolerance))
  {
    if (_must_exit == true)
    {
      ROS_INFO("Final waypoint reached. Exiting...\n");
      _twist.linear.x = 0;
      _twist.linear.y = 0;
      _twist.linear.z = 0;
      _twist.angular.x = 0;
      _twist.angular.y = 0;
      _twist.angular.z = 0;
      _waypoint_number = 0;
      _waypoints_received = false;  // Set it to false again, so to wait for new waypoints to serve
    }
    else
    {
      ROS_INFO("Error in accepted range. Next waypoint.\n");
      _current_goal = _waypoints.front();
      _waypoints.pop();  // Remove the element from the queue
      _waypoint_number += 1;
      _rise += 1;

      ROS_INFO("Next goal %d\n", _waypoint_number + 1);
      ROS_INFO("Coordinates (x,y,z, yaw) : (%f, %f, %f, %f)\n", _current_goal.translation.x,
               _current_goal.translation.y, _current_goal.translation.z, current_goal_yaw);
    }
  }

  if (_waypoint_number == _number_of_waypoints)
  {
    _waypoint_number = 0;
    _must_exit = true;
  }

  _vel_pub.publish(_twist);
}
}  // namespace navigate

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate");

  navigate::Navigator nav;

  ros::spin();
  return 0;
}
