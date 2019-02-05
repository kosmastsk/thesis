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
  // Initialize with zeros the message
  _odomMsg.header.frame_id = "map";
  _odomMsg.child_frame_id = "base_link";
  _odomMsg.pose.pose.position = {};
  _odomMsg.pose.pose.orientation = {};
  _odomMsg.pose.covariance = {};
  _odomMsg.twist.twist.linear = {};
  _odomMsg.twist.twist.angular = {};
  _odomMsg.twist.covariance = {};

  // Initialize the Subscriber
  _cmdVelListener = _nh.subscribe("/cmd_vel", 50, &Converter::cmdVelCallback, this);

  ros::Rate rate(5);  // 5hz

  // Initialize the Publisher
  _odomPublisher = _nh.advertise<nav_msgs::Odometry>("/odom", 50);
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
  // If this is the first time the callback is called, we do not want the whole interval that the node is running, so we
  // can use manually 0.5 seconds and then every time use the real time that passed between the calls using the
  // _lastTime variable
  if (firstCallFlag)
  {
    _lastTime = ros::Time::now();
    ros::Duration(0.1).sleep();
  }
  else
    firstCallFlag = 0;

  // Calculate the interval between the two calls
  ros::Duration delta_t = ros::Time::now() - _lastTime;

  float time_passed = delta_t.sec + delta_t.nsec * pow(10, -9);

  // Fill in the message
  _odomMsg.header.stamp = ros::Time::now();

  // Position
  _odomMsg.pose.pose.position.x += msg->linear.x * time_passed;
  _odomMsg.pose.pose.position.y += msg->linear.y * time_passed;
  _odomMsg.pose.pose.position.z += msg->linear.z * time_passed;

  // Orientation is a quaternion. angular velocity is rad/sec, multiply by sec and the input is in proper mode
  // Convert tf quaternion to std_msgs::quaternion to be accepted in the odom msg
  // geometry_msgs::Quaternion q;
  // quaternionTFToMsg(tf::createQuaternionFromRPY(msg->angular.x * time_passed, msg->angular.y * time_passed,
  // msg->angular.z * time_passed),
  // q);

  tf::Quaternion q;
  q = tf::createQuaternionFromRPY(0, 0, msg->angular.z);
  quaternionTFToMsg(q, _odomMsg.pose.pose.orientation);
  ROS_INFO_STREAM(q);
  // quaternionMsgToTF(_odomMsg.pose.pose.orientation, q);

  // tf::Quaternion q(0, 0, -1, msg->angular.z);
  // quaternionTFToMsg(q, _odomMsg.pose.pose.orientation);
  /*
    _odomMsg.pose.pose.orientation.x += msg->angular.x;
    _odomMsg.pose.pose.orientation.y += msg->angular.y;
    _odomMsg.pose.pose.orientation.z += msg->angular.z;
    _odomMsg.pose.pose.orientation.w = 1;
  */
  // Velocity
  // https://answers.ros.org/question/141871/why-is-there-a-twist-in-odometry-message/
  // TODO : Is there a way to calculate the measured one and not the desired????
  _odomMsg.twist.twist.linear = msg->linear;
  _odomMsg.twist.twist.angular = msg->angular;

  // Publish
  _odomPublisher.publish(_odomMsg);

  // Update time variable for next call
  _lastTime = ros::Time::now();
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
