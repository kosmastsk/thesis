/* This ROS node receives velocity values and converts them to Odometry for hector quadrotor */

#include "drone_gazebo/velocity_to_odom.h"

namespace vel_to_odom
{
bool firstCallFlag = 1;
/******************************/
/*        Constructor         */
/******************************/

Converter::Converter()
{
  ROS_INFO("Converter empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Converter::Converter(char* argv[])
{
  _outputFrame = std::string("world");
  _baseFrame = std::string("base_footprint");

  // Initialize with zeros the message
  _previousOdom.header.frame_id = _outputFrame;
  _previousOdom.child_frame_id = _baseFrame;

  // TODO parameter server
  _previousOdom.pose.pose.position.x = 1.0;
  _previousOdom.pose.pose.position.y = 0;
  _previousOdom.pose.pose.position.z = 0.2;

  _previousOdom.pose.pose.orientation.x = 0;
  _previousOdom.pose.pose.orientation.y = 0;
  _previousOdom.pose.pose.orientation.z = 0;
  _previousOdom.pose.pose.orientation.w = 1;

  _previousOdom.pose.covariance = {};
  _previousOdom.twist.twist.linear = {};
  _previousOdom.twist.twist.angular = {};
  _previousOdom.twist.covariance = {};

  // Initialize the Subscriber
  _cmdVelListener = _nh.subscribe("/cmd_vel", 50, &Converter::cmdVelCallback, this);

  // Initialize the Subscriber
  _heightListener = _nh.subscribe("/height", 50, &Converter::heightCallback, this);

  ros::Rate rate(50);  // hz

  // Initialize the Publisher
  _odomPublisher = _nh.advertise<nav_msgs::Odometry>("/odom", 10);
}

/******************************/
/*        Destructor          */
/******************************/

Converter::~Converter()
{
  ROS_INFO("Class Converter has been destroyed\n");
}

/******************************/
/*       cmdVelCallback       */
/******************************/

void Converter::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Create the odometry nav_msgs
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = _outputFrame;
  odom_msg.child_frame_id = _baseFrame;

  // If this is the first time the callback is called, we do not want the whole interval that the node is running, so we
  // can use manually 0.5 seconds and then every time use the real time that passed between the calls using the
  // _lastTime variable
  if (firstCallFlag)
  {
    _lastTime = ros::Time::now();
    ros::Duration(0.1).sleep();
    firstCallFlag = 0;
  }

  // Calculate the interval between the two calls
  double delta_t = ros::Time::now().toSec() - _lastTime.toSec();

  // Fill in the message
  odom_msg.header.stamp = ros::Time::now();

  // Position
  // Why this works is in the following links
  // http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePath.htm
  // http://rossum.sourceforge.net/papers/CalculationsForRobotics/CirclePathWithCalc.htm
  tf2::Quaternion temp_quat;
  tf2::fromMsg(getPreviousOdom().pose.pose.orientation, temp_quat);
  double theta = tf2::impl::getYaw(temp_quat);
  double s = msg->linear.x;
  double w = msg->angular.z;

  odom_msg.pose.pose.position.x =
      getPreviousOdom().pose.pose.position.x - (s / w) * sin(theta) + (s / w) * sin(w * delta_t + theta);
  odom_msg.pose.pose.position.y =
      getPreviousOdom().pose.pose.position.y + (s / w) * cos(theta) - (s / w) * cos(w * delta_t + theta);
  odom_msg.pose.pose.position.z = getHeight();

  // Special case when the robot is reaching is goal and not receiveing any more goals, the result is nan.

  if (std::isnan(odom_msg.pose.pose.position.x))
  {
    odom_msg.pose.pose.position.x = getPreviousOdom().pose.pose.position.x;
  }
  if (std::isnan(odom_msg.pose.pose.position.y))
  {
    odom_msg.pose.pose.position.y = getPreviousOdom().pose.pose.position.y;
  }

  // Orientation
  // Orientation is a quaternion. angular velocity is rad/sec
  // https://stackoverflow.com/questions/46908345/integrate-angular-velocity-as-quaternion-rotation
  tf2::Quaternion previous_orientation, rotation_quat;
  tf2::fromMsg(getPreviousOdom().pose.pose.orientation, previous_orientation);
  rotation_quat.setRPY(msg->angular.x * delta_t, msg->angular.y * delta_t,
                       msg->angular.z * delta_t);  // multiply with time to make it rads

  // Apply the rotation, normalize it and then convert tf2::quaternion to std_msgs::quaternion to be accepted in the
  // odom msg
  // https://stackoverflow.com/questions/46908345/integrate-angular-velocity-as-quaternion-rotation
  // TODO multiply with 0.5 SOMEHOW
  odom_msg.pose.pose.orientation = tf2::toMsg((rotation_quat * previous_orientation).normalize());

  // Velocity
  // https://answers.ros.org/question/141871/why-is-there-a-twist-in-odometry-message/
  // TODO : Is there a way to calculate the measured one and not the desired????
  odom_msg.twist.twist.linear = msg->linear;
  odom_msg.twist.twist.angular = msg->angular;

  // Publish
  _odomPublisher.publish(odom_msg);

  // Update the values for the next call
  setPreviousOdom(odom_msg);

  // Update time variable for next call
  _lastTime = odom_msg.header.stamp;  // ros::Time::now();
}

/******************************/
/*       heightCallback       */
/******************************/

void Converter::heightCallback(const std_msgs::Float64::ConstPtr& msg)
{
  // Provide the current height to the odometry topic
  setHeight(msg->data);
}

}  // namespace vel_to_odom

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_to_odom");

  vel_to_odom::Converter converter(argv);
  ros::spin();

  return 0;
}
