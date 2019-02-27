#include "particle_filter/MapModel.h"

MapModel::MapModel(ros::NodeHandle* nh)
{
  std::string mapService = "octomap_full";
  ROS_INFO("Requesting the map from %s...\n", nh->resolveName(mapService).c_str());

  octomap_msgs::GetOctomap::Request req;
  octomap_msgs::GetOctomap::Response res;

  while (nh->ok() && !ros::service::call(mapService, req, res))
  {
    ROS_INFO("Service call to %s failed. Trying again...\n", nh->resolveName(mapService).c_str());
    usleep(1000000);
  }

  octomap::ColorOcTree* colorTree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::fullMsgToMap(res.map));

  if (colorTree)
  {
    _map.reset(colorTree);
  }
  if (!_map || _map->size() <= 1)
  {
    ROS_ERROR("Map didn't retrieved,exiting");
    exit(-1);
  }

  double x, y, z;
  _map->getMetricSize(x, y, z);

  ROS_INFO("Occupancy map initialized with %zd nodes (%.2f x %.2f x %.2f m), %f m res.", _map->size(), x, y, z,
           _map->getResolution());
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
