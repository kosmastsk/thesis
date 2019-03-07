#ifndef MAPMODEL_H_
#define MAPMODEL_H_

#include <ros/ros.h>

#include <octomap/octomap_types.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MapModel
{
public:
  MapModel(ros::NodeHandle* nh);
  ~MapModel();

  // Return the map of the current model
  std::shared_ptr<octomap::ColorOcTree> getMap() const;

  // Check if a point is occupied in the octomap
  bool isOccupied(const octomap::point3d& point) const;

  // Check if a node is occupied in the octomap
  bool isOccupied(octomap::OcTreeNode* node) const;

protected:
  std::shared_ptr<octomap::ColorOcTree> _map;
  double _motionObstacleDist;
};

class OccupancyMap : public MapModel
{
public:
  OccupancyMap(ros::NodeHandle* nh);
  virtual ~OccupancyMap();

  bool isOccupied(octomap::OcTreeNode* node) const;

  pcl::PointCloud<pcl::PointXYZRGBA> toPCL();
};

#endif
