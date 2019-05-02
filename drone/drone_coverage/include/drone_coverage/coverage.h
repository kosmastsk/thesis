#ifndef COVERAGE_HEADER
#define COVERAGE_HEADER

#include "drone_coverage/graph_utils.h"

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "visualization_msgs/Marker.h"

#define DEGREE M_PI / 180

namespace drone_coverage
{
class Coverage
{
private:
  ros::NodeHandle _nh;

  ros::Subscriber _map_sub;
  ros::Subscriber _ogm_sub;
  ros::Publisher _covered_pub;
  ros::Publisher _vis_pub;
  ros::Publisher _waypoints_pub;

  // The pre-loaded octomap and the collection of 3d points
  octomap::OcTree* _octomap;
  octomap::OcTree* _covered;
  double _octomap_resolution;

  // The pre-loaded OGM
  nav_msgs::OccupancyGrid* _ogm;

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
  bool _ogm_loaded;

  Graph _graph;

  std::vector<bool> _discovered_nodes;

  // Callbacks
  void octomapCallback(const octomap_msgs::OctomapConstPtr& msg);
  void ogmCallback(const nav_msgs::OccupancyGridConstPtr& msg);

public:
  Coverage();
  ~Coverage();

  void calculateWaypoints();
  void postprocessWaypoints();
  void removeNonVisibleWaypoints();
  void reduceDimensionality();

  void calculateOrthogonalCoverage();
  void calculateCircularCoverage();
  float evaluateCoverage(octomap::OcTree* octomap, octomap::OcTree* covered);

  void findNeighbors(int root);
  bool safeCheckFrom2D(octomap::point3d sensor_position);
  bool findBestYaw(octomap::point3d sensor_position, double& yaw);
  double findCoverage(const octomap::point3d& wall_point, const octomap::point3d& direction);

  double proceedOneStep(double coord);

  void publishCoveredSurface();
  void visualizeWaypoints(std::vector<octomath::Pose6D> points);
  void publishWaypoints(std::vector<octomath::Pose6D> points);
};

}  // namespace drone_coverage

#endif
