#include <ros/ros.h>
#include <queue>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/impl/utils.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace navigate
{
class Navigator
{
private:
  ros::NodeHandle _nh;

  ros::Publisher _vel_pub;
  ros::Publisher _stamped_vel_pub;
  ros::Publisher _goal_reached_pub;

  ros::Subscriber _waypoints_sub;
  ros::Subscriber _pose_sub;

  visualization_msgs::MarkerArray _marker_array_msg;

  std::queue<geometry_msgs::Transform> _waypoints;
  int _number_of_waypoints;
  bool _waypoints_received;
  bool _hovering;

  geometry_msgs::Transform _current_goal;

  float _xy_tolerance;
  float _z_tolerance;
  float _yaw_tolerance;

  float _rot_max_speed;
  float _trans_max_speed;

  double _dt;

  // PID
  double _x_kp;
  double _x_ki;
  double _x_kd;

  double _y_kp;
  double _y_ki;
  double _y_kd;

  double _z_kp;
  double _z_ki;
  double _z_kd;

  double _attitude_kp;
  double _attitude_ki;
  double _attitude_kd;

  float _error_x;
  float _error_y;
  float _error_z;
  float _error_roll;
  float _error_pitch;
  float _error_yaw;

  float _prev_error_x;
  float _prev_error_y;
  float _prev_error_z;
  float _prev_error_roll;
  float _prev_error_pitch;
  float _prev_error_yaw;

  float _rise;

  float _proportional_x;
  float _proportional_y;
  float _proportional_z;
  float _proportional_roll;
  float _proportional_pitch;
  float _proportional_yaw;

  float _integral_x;
  float _integral_y;
  float _integral_z;
  float _integral_roll;
  float _integral_pitch;
  float _integral_yaw;

  float _derivative_x;
  float _derivative_y;
  float _derivative_z;
  float _derivative_roll;
  float _derivative_pitch;
  float _derivative_yaw;

  float _action_x;
  float _action_y;
  float _action_z;
  float _action_roll;
  float _action_pitch;
  float _action_yaw;

  geometry_msgs::Transform _pose;
  geometry_msgs::Twist _twist;
  geometry_msgs::TwistStamped _twist_stamped;

  bool _must_exit;
  int _waypoint_number;

  // Callbacks
  void waypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // We do not want the drone to get really high values of speed, either positive or negative. So we need to clamp it in
  // the range [-max_action, +max_action]
  void clamp(float& action, float max_action);
  bool controlYaw(double current_goal_yaw, double pose_yaw, double dt);
  bool controlXY(double current_goal_x, double pose_x, double current_goal_y, double pose_y, double dt,
                 double pose_yaw);

public:
  Navigator();
  ~Navigator();
};

}  // namespace navigate
