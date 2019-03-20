#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

void callback(const geometry_msgs::PoseStampedConstPtr& amcl_pose, const nav_msgs::OdometryConstPtr& gnd_state)
{
  double error_x, error_y, error_z;
  error_x = amcl_pose->pose.position.x - gnd_state->pose.pose.position.x;
  error_y = amcl_pose->pose.position.y - gnd_state->pose.pose.position.y;
  error_z = amcl_pose->pose.position.z - gnd_state->pose.pose.position.z;

  ROS_INFO_ONCE("[/amcl_pose] - [/ground_truth/state]\n");
  ROS_INFO("Error in [x, y, z] : [%f, %f, %f]\n", error_x, error_y, error_z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "error_detector");

  ros::NodeHandle nh;

  message_filters::Subscriber<geometry_msgs::PoseStamped> amcl_sub(nh, "/amcl_pose", 1);
  message_filters::Subscriber<nav_msgs::Odometry> gnd_sub(nh, "/ground_truth/state", 1);
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amcl_sub, gnd_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
