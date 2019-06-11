#ifndef ONLINE_COVERAGE_HEADER
#define ONLINE_COVERAGE_HEADER

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "drone_gazebo/Float64Stamped.h"

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#define DEGREE M_PI / 180

namespace drone_coverage
{
class OnlineCoverage
{
private:
  ros::NodeHandle _nh;

  ros::Subscriber _map_sub;
  ros::Subscriber _pose_sub;
  ros::Publisher _covered_pub;
  ros::Publisher _percentage_pub;
  ros::Publisher _volume_pub;

  // The pre-loaded octomap and the collection of 3d points
  octomap::OcTree* _octomap;
  octomap::ColorOcTree* _covered;
  double _octomap_resolution;
  float _octomap_volume;

  double _rfid_range;
  double _rfid_hfov;
  double _rfid_vfov;
  std::string _sensor_shape;

  double _rfid_direction_x;
  double _rfid_direction_y;
  double _rfid_direction_z;

  double _min_obstacle_height;
  double _max_obstacle_height;

  bool _octomap_loaded;

  // Callbacks
  void octomapCallback(const octomap_msgs::OctomapConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

public:
  OnlineCoverage();
  ~OnlineCoverage();

  void calculateOrthogonalCoverage(const geometry_msgs::Pose);
  void calculateCircularCoverage(const geometry_msgs::Pose);

  float calculateOccupiedVolume(octomap::ColorOcTree* octomap);
  float calculateOccupiedVolume(octomap::OcTree* octomap);
  void publishCoveredSurface();
  void publishPercentage();
};

}  // namespace drone_coverage

#endif
