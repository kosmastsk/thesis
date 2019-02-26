/* particle_filter_node.cpp */

#include "particle_filter/particle_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "particle_filter_node");

  pf::Particles particles;

  ros::spin();

  return 0;
}
