#ifndef WALL_FINDER_HEADER
#define WALL_FINDER_HEADER

#include <iostream>
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include "visualization_msgs/Marker.h"

#define DEGREE M_PI / 180

namespace drone_coverage
{
class CoverageFinder
{
private:
  ros::NodeHandle _nh;

  ros::Subscriber _map_sub;
  ros::Publisher _covered_pub;
  ros::Publisher _vis_pub;

  // The pre-loaded octomap and the collection of 3d points
  octomap::OcTree* _octomap;
  octomap::OcTree* _walls;
  double _octomap_resolution;

  // Keep all points in a vector
  std::vector<octomath::Pose6D> _points;

  double _min_bounds[3];
  double _max_bounds[3];
  double _init_pose[3];

  double _uav_radius;
  double _uav_safety_offset;

  double _rfid_range;
  double _rfid_hfov;
  double _rfid_vfov;

  double _min_obstacle_height;

  octomap::point3d _sensor_position;

  bool _octomap_loaded;

  // Callbacks
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

public:
  CoverageFinder();
  ~CoverageFinder();

  void findCoveredSurface();
  void publishCoveredSurface();
  void publishWaypoints();
  bool safeCheck(octomap::point3d wall_point, octomap::point3d sensor_position);
};

}  // namespace drone_coverage

#endif
