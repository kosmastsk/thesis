#include "drone_3d_nav/navigate.h"

namespace navigate
{
Navigator::Navigator()
{
  // Parameters
  _f = boost::bind(&Navigator::dynamicParamCallback, this, _1, _2);
  _server.setCallback(_f);

  _must_exit = false;
  _waypoint_number = 0;
  _waypoints_received = false;
  _dt = 0;
  _hovering = false;

  _error_x = 0;
  _error_y = 0;
  _error_z = 0;
  _error_roll = 0;
  _error_pitch = 0;
  _error_yaw = 0;

  _prev_error_x = 0;
  _prev_error_y = 0;
  _prev_error_z = 0;
  _prev_error_roll = 0;
  _prev_error_pitch = 0;
  _prev_error_yaw = 0;

  _proportional_x = 0;
  _proportional_y = 0;
  _proportional_z = 0;
  _proportional_roll = 0;
  _proportional_pitch = 0;
  _proportional_yaw = 0;

  _integral_x = 0;
  _integral_y = 0;
  _integral_z = 0;
  _integral_roll = 0;
  _integral_pitch = 0;
  _integral_yaw = 0;

  _derivative_x = 0;
  _derivative_y = 0;
  _derivative_z = 0;
  _derivative_roll = 0;
  _derivative_pitch = 0;
  _derivative_yaw = 0;

  _action_x = 0;
  _action_y = 0;
  _action_z = 0;
  _action_roll = 0;
  _action_pitch = 0;
  _action_yaw = 0;

  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  _stamped_vel_pub = _nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel/stamped", 5);
  _goal_reached_pub = _nh.advertise<std_msgs::Bool>("/goal_reached", 1);

  // amcl_pose
  _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/amcl_pose", 5, &Navigator::poseCallback, this);
  _waypoints_sub = _nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/waypoints_smooth", 1,
                                                                           &Navigator::waypointCallback, this);
}

Navigator::~Navigator()
{
}

void Navigator::dynamicParamCallback(drone_3d_nav::pidConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request!\n");

  _x_kp = config.x_kp;
  _x_ki = config.x_ki;
  _x_kd = config.x_kd;

  _y_kp = config.y_kp;
  _y_ki = config.y_ki;
  _y_kd = config.y_kd;

  _z_kp = config.z_kp;
  _z_ki = config.z_ki;
  _z_kd = config.z_kd;

  _attitude_kp = config.attitude_kp;
  _attitude_ki = config.attitude_ki;
  _attitude_kd = config.attitude_kd;

  _xy_tolerance = config.xy_tolerance;
  _z_tolerance = config.z_tolerance;
  _yaw_tolerance = config.yaw_tolerance;

  _xy_max_speed = config.xy_max_speed;
  _z_max_speed = config.z_max_speed;
  _rot_max_speed = config.rot_max_speed;
}

void Navigator::waypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  // The first item in the points array is the initial position
  _waypoint_number = 1;
  _number_of_waypoints = msg->points.size();
  // Empty the queue for new waypoints
  while (!_waypoints.empty())
  {
    _waypoints.pop();
  }

  ROS_INFO("[Navigate] %d waypoints received\n", _number_of_waypoints);
  for (unsigned int i = 0; i < _number_of_waypoints; i++)
  {
    _waypoints.push(msg->points[i].transforms[0]);
  }

  _current_goal = _waypoints.front();
  _waypoints.pop();  // Remove this element from the queue

  _waypoints_received = true;
  _must_exit = false;
  _hovering = false;
}

void Navigator::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // If the waypoints have not received, we cannot proceed with the navigation.
  if (!_waypoints_received && !_hovering)
  {
    ROS_WARN_ONCE("[Navigate] Waypoints not received. Skipping current pose...\n");
    return;
  }

  _pose.translation.x = msg->pose.position.x;
  _pose.translation.y = msg->pose.position.y;
  _pose.translation.z = msg->pose.position.z;

  _pose.rotation = msg->pose.orientation;  // geometry_msgs::Quaternion showing the current orientation

  // Get the Yaw from current goal and odometry measurement
  tf2::Quaternion current_goal_quat, pose_quat;
  tf2::fromMsg(_current_goal.rotation, current_goal_quat);
  tf2::fromMsg(_pose.rotation, pose_quat);

  // If we want to find roll and pitch, we need Matrix3x3 and getRPY
  double pose_roll, pose_pitch, pose_yaw;
  tf2::impl::getEulerYPR(pose_quat, pose_yaw, pose_pitch, pose_roll);

  double current_goal_yaw = tf2::impl::getYaw(current_goal_quat);

  _dt = ros::Time::now().toSec() - _dt;

  // Find errors and PID values to calculate next action
  _error_x = _current_goal.translation.x - _pose.translation.x;
  _error_y = _current_goal.translation.y - _pose.translation.y;
  _error_z = _current_goal.translation.z - _pose.translation.z;
  // Desired roll and pitch are 0
  _error_roll = 0 - pose_roll;
  _error_pitch = 0 - pose_pitch;

  _proportional_x = _x_kp * _error_x;
  _proportional_y = _y_kp * _error_y;
  _proportional_z = _z_kp * _error_z;
  _proportional_roll = _attitude_kp * _error_roll;
  _proportional_pitch = _attitude_kp * _error_pitch;

  _integral_x = _x_ki * (_integral_x + _error_x * _dt);
  _integral_y = _y_ki * (_integral_y + _error_y * _dt);
  _integral_z = _z_ki * (_integral_z + _error_z * _dt);
  _integral_roll = _attitude_ki * (_integral_roll + _error_roll * _dt);
  _integral_pitch = _attitude_ki * (_integral_pitch + _error_pitch * _dt);

  _derivative_x = _x_kd * ((_error_x - _prev_error_x) / _dt);
  _derivative_y = _y_kd * ((_error_y - _prev_error_y) / _dt);
  _derivative_z = _z_kd * ((_error_z - _prev_error_z) / _dt);
  _derivative_roll = _attitude_kd * ((_error_roll - _prev_error_roll) / _dt);
  _derivative_pitch = _attitude_kd * ((_error_pitch - _prev_error_pitch) / _dt);

  _prev_error_x = _error_x;
  _prev_error_y = _error_y;
  _prev_error_z = _error_z;
  _prev_error_roll = _error_roll;
  _prev_error_pitch = _error_pitch;

  _action_x = _proportional_x + _integral_x + _derivative_x;
  _action_y = _proportional_y + _integral_y + _derivative_y;
  _action_z = _proportional_z + _integral_z + _derivative_z;
  _action_roll = _proportional_roll + _integral_roll + _derivative_roll;
  _action_pitch = _proportional_pitch + _integral_pitch + _derivative_pitch;

  // converting from world frame to drone frame
  // Rotate with Rodriguez formula
  float final_action_x = _action_x * cos(pose_yaw) + _action_y * sin(pose_yaw);
  float final_action_y = _action_y * cos(pose_yaw) - _action_x * sin(pose_yaw);

  // Clamp the velocities
  clamp(final_action_x, _xy_max_speed);
  clamp(final_action_y, _xy_max_speed);
  clamp(_action_z, _z_max_speed);
  clamp(_action_roll, _rot_max_speed);
  clamp(_action_pitch, _rot_max_speed);

  _twist.linear.x = final_action_x;
  _twist.linear.y = final_action_y;
  _twist.linear.z = _action_z;
  _twist.angular.x = _action_roll;
  _twist.angular.y = _action_pitch;
  _twist.angular.z = 0;

  // Ensure that the drone's position is in accepted range error
  if (fabs(_error_x) <= _xy_tolerance && fabs(_error_y) <= _xy_tolerance && fabs(_error_z) <= _z_tolerance)
  {
    if (controlYaw(current_goal_yaw, pose_yaw))
    {
      if (_must_exit && !_hovering)
      {
        ROS_INFO("[Navigate] Final waypoint reached. Hovering...\n");
        _hovering = true;
        _waypoints_received = false;  // Set it to false again, so to wait for new waypoints to serve
        std_msgs::Bool feedback;
        feedback.data = true;
        // Provide feedback for the next waypoint to reach
        _goal_reached_pub.publish(feedback);
      }
      else if (!_hovering)
      {
        ROS_INFO("[Navigate] Error in accepted range. Next waypoint.\n");
        _current_goal = _waypoints.front();
        _waypoints.pop();  // Remove the element from the queue
        _waypoint_number += 1;

        ROS_INFO("[Navigate] Next goal %d\n", _waypoint_number);
        ROS_INFO("[Navigate] Coordinates (x,y,z, yaw) : (%f, %f, %f, %f)\n", _current_goal.translation.x,
                 _current_goal.translation.y, _current_goal.translation.z, current_goal_yaw);
        // Do not publish twist that are related to the waypoint, since we can now proceed to waypoint + 1
        return;
      }
    }
  }

  if (_waypoint_number == _number_of_waypoints)
    _must_exit = true;

  _twist_stamped.twist = _twist;
  _twist_stamped.header.stamp = ros::Time::now();

  _vel_pub.publish(_twist);
  _stamped_vel_pub.publish(_twist_stamped);

  ROS_DEBUG("Error (X, Y, Z, Yaw) : (%0.2f, %0.2f, %0.2f, %0.2f) \n", _error_x, _error_y, _error_z, _error_yaw);
  ROS_DEBUG("Action (X, Y, Z, Yaw) : (%0.2f, %0.2f, %0.2f, %0.2f) \n", _action_x, _action_y, _action_z, _action_yaw);

  // Create some minor delay
  ros::Duration(0.001).sleep();
}

bool Navigator::controlYaw(float current_goal_yaw, float pose_yaw)
{
  _error_yaw = current_goal_yaw - pose_yaw;

  if (fabs(_error_yaw) < _yaw_tolerance)
    return true;
  else
  {
    while (fabs(_error_yaw) > M_PI)
      _error_yaw = _error_yaw - ((_error_yaw > 0) - (_error_yaw < 0)) * M_PI;

    _proportional_yaw = _attitude_kp * _error_yaw;
    _integral_yaw = _attitude_ki * (_integral_yaw + _error_yaw * _dt);
    _derivative_yaw = _attitude_kd * (_error_yaw - _prev_error_yaw) / _dt;
    _prev_error_yaw = _error_yaw;
    _action_yaw = _proportional_yaw + _integral_yaw + _derivative_yaw;
    clamp(_action_yaw, _rot_max_speed);

    _twist.linear.x = 0;
    _twist.linear.y = 0;
    _twist.linear.z = 0;
    _twist.angular.x = 0;
    _twist.angular.y = 0;
    _twist.angular.z = _action_yaw;
  }
  return false;
}

void Navigator::clamp(float& action, float max_action)
{
  if (fabs(action) > max_action)
  {
    // The first () gives us the sign of the action. If it is greater than max, then give it the max value
    action = ((action > 0) - (action < 0)) * max_action;
  }
  return;
}

}  // namespace navigate

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate");

  navigate::Navigator nav;

  ros::spin();
  return 0;
}
