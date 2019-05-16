#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

void visualizeWaypoints(std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> points)
{
  ros::NodeHandle nr;
  ros::Publisher vis_pub = nr.advertise<visualization_msgs::Marker>("/visualization_marker", 25, true);
  for (std::size_t idx = 0; idx < points.size(); idx++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "trajectory";
    marker.id = idx;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = points.at(idx).transforms[0].translation.x;
    marker.pose.position.y = points.at(idx).transforms[0].translation.y;
    marker.pose.position.z = points.at(idx).transforms[0].translation.z;
    marker.pose.orientation.x = points.at(idx).transforms[0].rotation.x;
    marker.pose.orientation.y = points.at(idx).transforms[0].rotation.y;
    marker.pose.orientation.z = points.at(idx).transforms[0].rotation.z;
    marker.pose.orientation.w = points.at(idx).transforms[0].rotation.w;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = points.at(idx).transforms[0].translation.z / 2;
    vis_pub.publish(marker);
    ros::Duration(0.001).sleep();
  }
}

std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> createStraightLine(int number_of_points)
{
  std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> points;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.transforms.resize(1);
  geometry_msgs::Transform transform;

  // Point #1
  transform.translation.x = 9;
  transform.translation.y = 0;
  transform.translation.z = 1.2;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0;
  transform.rotation.w = 1;
  point.transforms[0] = transform;
  points.push_back(point);

  // Ended
  visualizeWaypoints(points);
  return points;
}

std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> createSpiral(int number_of_points)
{
  std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> points;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.transforms.resize(1);
  geometry_msgs::Transform transform;

  double max_height = 1.5;
  float angle = 0;
  int a = 1, b = 1;

  for (int i = 0; i < number_of_points; i++)
  {
    angle = 0.05 * i;
    transform.translation.x = (a + b * angle) * cos(angle * M_PI * 2);
    transform.translation.y = (a + b * angle) * sin(angle * M_PI * 2);
    transform.translation.z = 0.5 + float(i) / float(number_of_points) * max_height;
    transform.rotation.x = 0;
    transform.rotation.y = 0;
    transform.rotation.z = 0;
    transform.rotation.w = 1;
    point.transforms[0] = transform;
    points.push_back(point);
  }
  visualizeWaypoints(points);
  return points;
}

std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> createMeander(int number_of_points)
{
  std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> points;
  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  point.transforms.resize(1);
  geometry_msgs::Transform transform;

  // Point #1
  transform.translation.x = 0;
  transform.translation.y = -3;
  transform.translation.z = 0.5;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0;
  transform.rotation.w = 1;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #2
  transform.translation.x = 5;
  transform.translation.y = -3;
  transform.translation.z = 0.5;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0.7071068;
  transform.rotation.w = 0.7071068;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #3
  transform.translation.x = 5;
  transform.translation.y = 3;
  transform.translation.z = 0.5;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 1;
  transform.rotation.w = 0;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #4
  transform.translation.x = 1;
  transform.translation.y = 3;
  transform.translation.z = 0.5;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = -0.7071068;
  transform.rotation.w = 0.7071068;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #5
  transform.translation.x = 1;
  transform.translation.y = -2;
  transform.translation.z = 1;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0;
  transform.rotation.w = 1;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #6
  transform.translation.x = 3;
  transform.translation.y = -2;
  transform.translation.z = 1;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0.7071068;
  transform.rotation.w = 0.7071068;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #7
  transform.translation.x = 3;
  transform.translation.y = 2;
  transform.translation.z = 1;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 1;
  transform.rotation.w = 0;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #8
  transform.translation.x = 2;
  transform.translation.y = 2;
  transform.translation.z = 1;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = -0.7071068;
  transform.rotation.w = 0.7071068;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #9
  transform.translation.x = 2;
  transform.translation.y = 0;
  transform.translation.z = 2;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0;
  transform.rotation.w = 1;
  point.transforms[0] = transform;
  points.push_back(point);

  // Point #10
  transform.translation.x = 2.5;
  transform.translation.y = 0;
  transform.translation.z = 2;
  transform.rotation.x = 0;
  transform.rotation.y = 0;
  transform.rotation.z = 0;
  transform.rotation.w = 1;
  point.transforms[0] = transform;
  points.push_back(point);

  // Ended
  visualizeWaypoints(points);
  return points;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "produce_trajectory");

  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/waypoints_smooth", 10, true);

  trajectory_msgs::MultiDOFJointTrajectory msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  std::string type = "spiral";

  if (type == "straight_line")
    msg.points = createStraightLine(1);
  else if (type == "spiral")
  {
    msg.points = createSpiral(25);
  }
  else if (type == "meander")
  {
    msg.points = createMeander(10);
  }

  trajectory_pub.publish(msg);

  ros::spin();
  return 0;
}
