#ifndef WALL_FINDER_HEADER
#define WALL_FINDER_HEADER

#include <iostream>
#include <queue>

#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

namespace drone_coverage
{
class WallFinder
{
private:
  ros::NodeHandle _nh;

  ros::Subscriber _map_sub;

  // The pre-loaded octomap and the collection of 3d points
  octomap::OcTree* _octomap;
  octomap::Pointcloud _points;

  // Bounds
  double _min_bounds[3];
  double _max_bounds[3];

  // Callbacks
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

public:
  WallFinder();
  ~WallFinder();
};

}  // namespace drone_coverage

#endif
