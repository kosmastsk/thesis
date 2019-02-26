#ifndef MAPMODEL_H_
#define MAPMODEL_H_

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <ros/ros.h>

class MapModel
{
public:
  MapModel(ros::NodeHandle* nh);
  ~MapModel();

  // Return the map of the current model
  nav_msgs::OccupancyGrid getMap() const;

  // Check if a point is occupied in the OGM
  bool isOccupied(int x, int y) const;

protected:
  nav_msgs::OccupancyGrid _map;

private:
};

#endif
