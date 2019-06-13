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
