#include "drone_2d_nav/control.h"

namespace control
{
/******************************/
/*        Constructor         */
/******************************/

Controller::Controller()
{
  ROS_INFO("Controller empty object created");
}

/******************************/
/* Constructor with arguments */
/******************************/

Controller::Controller(char* argv[])
{
  // Make the desired height a parameter
  setDesiredHeight(atoi(argv[1]));

  ros::Rate loop_rate(100);

  // Initialize the Subscriber
  _heightListener = _nh.subscribe("/height", 1, &Controller::heightCallback, this);

  // Initialize the Subscriber
  _moveBaseListener = _nh.subscribe("/cmd_vel/move_base", 1, &Controller::moveBaseCallback, this);

  // Initialize the Publisher
  _cmdvelPublisher = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

/******************************/
/*        Destructor          */
/******************************/

Controller::~Controller()
{
  ROS_INFO("Class Controller has been destroyed\n");
}

/******************************/
/*       heightCallback       */
/******************************/

void Controller::heightCallback(const drone_gazebo::Float64Stamped::ConstPtr& msg)
{
  float currentHeight = msg->data;

  float z;  // distance to cover

  // z = v * t; // t: sec, z: m, v = m/s
  z = getDesiredHeight() - currentHeight;
  z = roundf(z * 100) / 100;  // round to nearest

  // z becomes velocity now
  z = z * 100 / 90;  // Convert z to cm and divide with time

  setZLinear(z);
}

/******************************/
/*      moveBaseCallback      */
/******************************/

void Controller::moveBaseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Create the Twist message that will be published with all the velocities
  geometry_msgs::Twist velMsg;

  velMsg.linear.x = msg->linear.x;
  velMsg.linear.y = msg->linear.y;
  velMsg.linear.z = getZLinear();

  velMsg.angular.x = msg->angular.x;
  velMsg.angular.y = msg->angular.y;
  velMsg.angular.z = msg->angular.z;

  _cmdvelPublisher.publish(velMsg);
}

}  // namespace control`
