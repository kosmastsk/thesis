/* This ROS node receives velocity values and converts them to Odometry for hector quadrotor */

#include "drone_gazebo/ground_truth_for_odom.h"

namespace gnd_for_odom
{
bool firstCallFlag = 1;
/******************************/
/*        Constructor         */
/******************************/

Gnd::Gnd()
{
  ROS_INFO("Gnd empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Gnd::Gnd(char* argv[])
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
  _gndListener = _nh.subscribe("/ground_truth/state", 50, &Gnd::gndCallback, this);

  ros::Rate rate(100);  // hz

  // Initialize the Publisher
  _odomPublisher = _nh.advertise<nav_msgs::Odometry>("/odom", 10);
}

/******************************/
/*        Destructor          */
/******************************/

Gnd::~Gnd()
{
  ROS_INFO("Class Gnd has been destroyed\n");
}

/******************************/
/*       cmdVelCallback       */
/******************************/

void Gnd::gndCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // Publish the frame transform
  geometry_msgs::TransformStamped temp_tf;

  temp_tf.header.stamp = msg->header.stamp;
  temp_tf.header.frame_id = _outputFrame;
  temp_tf.child_frame_id = _baseFrame;

  temp_tf.transform.rotation = msg->pose.pose.orientation;  // TODO find the difference

  temp_tf.transform.translation.x = msg->pose.pose.position.x - _previousOdom.pose.pose.position.x;
  temp_tf.transform.translation.y = msg->pose.pose.position.y - _previousOdom.pose.pose.position.y;
  temp_tf.transform.translation.z = 0;  // msg->pose.pose.position.z - _previousOdom.pose.pose.position.z;

  _tfBroadcaster.sendTransform(temp_tf);

  // Publish
  _previousOdom = *msg;
  _odomPublisher.publish(msg);
}

}  // namespace vel_to_odom

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gnd_for_odom");

  gnd_for_odom::Gnd odom(argv);
  ros::spin();

  return 0;
}
