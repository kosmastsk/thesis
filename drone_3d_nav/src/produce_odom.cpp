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
/* This ROS node receives velocity values and converts them to Odometry for hector quadrotor */

#include "drone_3d_nav/produce_odom.h"

namespace produce_odom
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
  // Initialize the Subscribers
  _height_listener = new message_filters::Subscriber<drone_gazebo::Float64Stamped>(_nh, "/height", 100);
  _imu_listener = new message_filters::Subscriber<sensor_msgs::Imu>(_nh, "/raw_imu", 100);
  _velocity_listener = new message_filters::Subscriber<geometry_msgs::TwistStamped>(_nh, "/cmd_vel/stamped", 100);

  // Using the ApproximateTime Policy
  // http://wiki.ros.org/message_filters/ApproximateTime
  typedef message_filters::sync_policies::ApproximateTime<drone_gazebo::Float64Stamped, sensor_msgs::Imu,
                                                          geometry_msgs::TwistStamped>
      MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), *_height_listener, *_imu_listener,
                                                   *_velocity_listener);

  sync.registerCallback(boost::bind(&Converter::syncedCallback, this, _1, _2, _3));

  // Set frames for the new message
  _outputFrame = std::string("map");
  _baseFrame = std::string("base_footprint");

  // Initialize the message using parameters
  _previousOdom.header.frame_id = _outputFrame;
  _previousOdom.child_frame_id = _baseFrame;

  _nh.param<double>("/x_pos", _previousOdom.pose.pose.position.x, 0);
  _nh.param<double>("/y_pos", _previousOdom.pose.pose.position.y, 0);
  _nh.param<double>("/z_pos", _previousOdom.pose.pose.position.z, 0);

  _previousOdom.pose.pose.orientation.x = 0;
  _previousOdom.pose.pose.orientation.y = 0;
  _previousOdom.pose.pose.orientation.z = 0;
  _previousOdom.pose.pose.orientation.w = 1;

  _previousOdom.pose.covariance = {};
  _previousOdom.twist.twist.linear = {};
  _previousOdom.twist.twist.angular = {};
  _previousOdom.twist.covariance = {};

  // Initialize the Publisher
  _odomPublisher = _nh.advertise<nav_msgs::Odometry>("/odom", 10);

  // If there are no cmd_vel commands, the odometry needs to be published, even if it is unchanged
  publishOdometry();
}

/******************************/
/*        Destructor          */
/******************************/

Converter::~Converter()
{
  ROS_INFO("Class Converter has been destroyed\n");
}

/******************************/
/*      syncedCallback        */
/******************************/

void Converter::syncedCallback(const drone_gazebo::Float64StampedConstPtr& height, const sensor_msgs::ImuConstPtr& imu,
                               const geometry_msgs::TwistStampedConstPtr& velocity)
{
  // Check the element 0 of orientation covariance. If it is -1, it means that the IMU is not producing orientation
  // estimate, so we need to disregard the associated estimate
  if (imu->orientation_covariance[0] == -1)
  {
    ROS_WARN("IMU is not producing orientation estimate. orientation_covariance[0] == 1\n");
    return;
  }

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
  double delta_t = (ros::Time::now() - _lastTime).toSec();

  // Get the yaw value from the imu
  tf2::Quaternion temp_quat;
  tf2::fromMsg(imu->orientation, temp_quat);
  double yaw = tf2::impl::getYaw(temp_quat);

  // Fill in the message
  odom_msg.header.stamp = ros::Time::now();

  // Position
  // x += (cos(yaw)*vx - sin(yaw)*vy) * dt
  // y += (sin(yaw)*vx + cos(yaw)*vy) * dt
  // https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/
  // http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
  odom_msg.pose.pose.position.x = getPreviousOdom().pose.pose.position.x +
                                  (velocity->twist.linear.x * cos(yaw) - velocity->twist.linear.y * sin(yaw)) * delta_t;
  odom_msg.pose.pose.position.y = getPreviousOdom().pose.pose.position.y +
                                  (velocity->twist.linear.x * sin(yaw) + velocity->twist.linear.y * cos(yaw)) * delta_t;
  odom_msg.pose.pose.position.z = height->data;

  // Orientation is provided directly from the imu
  odom_msg.pose.pose.orientation = imu->orientation;

  // Velocity
  // https://answers.ros.org/question/141871/why-is-there-a-twist-in-odometry-message/
  odom_msg.twist.twist.linear = velocity->twist.linear;
  odom_msg.twist.twist.angular = velocity->twist.angular;

  // Update the values for the next call
  setPreviousOdom(odom_msg);

  // Update time variable for next call
  _lastTime = odom_msg.header.stamp;
}

/******************************/
/*      publishOdometry       */
/******************************/

void Converter::publishOdometry()
{
  ros::Rate rate(100);  // hz

  while (ros::ok())
  {
    updateOdomTime(ros::Time::now());
    _odomPublisher.publish(getPreviousOdom());
    ros::spinOnce();

    rate.sleep();
  }
}

}  // namespace vel_to_odom

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "produce_odom");

  produce_odom::Converter converter(argv);
  ros::spinOnce();

  return 0;
}
