#include "drone_coverage/wall_finder.h"

namespace drone_coverage
{
WallFinder::WallFinder()
{
  ROS_INFO("Wall Finder object created\n");
  _map_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &WallFinder::octomapCallback, this);
}

WallFinder::~WallFinder()
{
}

void WallFinder::octomapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  // Load octomap msg
  octomap::AbstractOcTree* abstract = octomap_msgs::msgToMap(*msg);
  if (abstract)
  {
    octomap::ColorOcTree* coloroctree = dynamic_cast<octomap::ColorOcTree*>(abstract);
    _octomap = reinterpret_cast<octomap::OcTree*>(coloroctree);

    if (_octomap == NULL)
    {
      ROS_WARN("Octomap message does not contain an OcTree\n");
    }
    else
    {
      ROS_INFO("Octomap successfully loaded\n");
    }
  }
  else
  {
    ROS_WARN("Could not deserialize message to OcTree");
  }

  _octomap->getMetricMin(_min_bounds[0], _min_bounds[1], _min_bounds[2]);
  _octomap->getMetricMax(_max_bounds[0], _max_bounds[1], _max_bounds[2]);

  // We do now want points that are under the ground --> bound < 0 -->convert them to 0
  // TODO

  ROS_INFO("Octomap bounds are (x,y,z) : \n [min]  %f, %f, %f\n [max]  %f, %f, %f", _min_bounds[0], _min_bounds[1],
           _min_bounds[2], _max_bounds[0], _max_bounds[1], _max_bounds[2]);
}

}  // namespace drone_coverage
