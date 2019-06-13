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
#include "particle_filter/MapModel.h"

MapModel::MapModel(ros::NodeHandle* nh)
{
  _motionObstacleDist = 0.2;
}

MapModel::~MapModel()
{
}

std::shared_ptr<octomap::ColorOcTree> MapModel::getMap() const
{
  return _map;
}

bool MapModel::isOccupied(const octomap::point3d& point) const
{
  octomap::OcTreeNode* ocNode = _map->search(point);
  if (ocNode)
  {
    return isOccupied(ocNode);
  }
  else
    return false;
}

bool MapModel::isOccupied(octomap::OcTreeNode* node) const
{
  return _map->isNodeOccupied(node);
}

/* Occupancy Grid Map */
OccupancyMap::OccupancyMap(ros::NodeHandle* nh) : MapModel(nh)
{
  std::string srvName = "octomap_full";
  ROS_INFO("Requesting the map from %s...", nh->resolveName(srvName).c_str());
  octomap_msgs::GetOctomap::Request req;
  octomap_msgs::GetOctomap::Response resp;

  while (nh->ok() && !ros::service::call(srvName, req, resp))
  {
    ROS_WARN("Request to %s failed; trying again...", nh->resolveName(srvName).c_str());
    usleep(1000000);
  }

  octomap::ColorOcTree* colorTree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::fullMsgToMap(resp.map));

  _map.reset(colorTree);
  if (!_map && (_map->size() <= 1))
  {
    ROS_ERROR("Map didn't retrieved,exiting");
    exit(-1);
  }
  double x, y, z;
  _map->getMetricSize(x, y, z);
  ROS_INFO("Occupancy map initialized with %zd nodes (%.2f x %.2f x %.2f m), %f m res.", _map->size(), x, y, z,
           _map->getResolution());
}

OccupancyMap::~OccupancyMap()
{
}

bool OccupancyMap::isOccupied(octomap::OcTreeNode* node) const
{
  return _map->isNodeOccupied(node);
}

pcl::PointCloud<pcl::PointXYZRGBA> OccupancyMap::toPCL()
{
  octomap::ColorOcTree::leaf_iterator itleaf = _map->begin_leafs();
  octomap::ColorOcTree::leaf_iterator endleaf = _map->end_leafs();
  pcl::PointCloud<pcl::PointXYZRGBA> octoMapFullPointCloud;
  for (; itleaf != endleaf; ++itleaf)
  {
    pcl::PointXYZRGBA tmp;
    tmp.x = itleaf.getX();
    tmp.y = itleaf.getY();
    tmp.z = itleaf.getZ();
    tmp.r = itleaf->getColor().r;
    tmp.g = itleaf->getColor().g;
    tmp.b = itleaf->getColor().b;
    octoMapFullPointCloud.push_back(tmp);
  }
  return octoMapFullPointCloud;
}
