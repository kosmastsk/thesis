#include "particle_filter/MapModel.h"

MapModel::MapModel(ros::NodeHandle* nh)
{
  std::string mapService = "map";
  ROS_INFO("Requesting the map from %s.\n", mapService.c_str());
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (nh->ok() && !ros::service::call(mapService, req, res))
  {
    ROS_INFO("Service call to %s failed. Trying again...\n", mapService.c_str());
  }

  _map = res.map;
  ROS_INFO("Map received with: resolution: %f, width: %d, height: %d\n", _map.info.resolution, _map.info.width,
           _map.info.height);
}

MapModel::~MapModel()
{
}

nav_msgs::OccupancyGrid MapModel::getMap() const
{
  return _map;
}

bool MapModel::isOccupied(int x, int y) const
{
  float possibility = _map.data[x + _map.info.width * y];
  if (possibility > 0.5)
    return true;
  else
    return false;
}
