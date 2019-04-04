#include <drone_coverage/wall_finder.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_coverage_node");

  drone_coverage::WallFinder finder;

  ros::spin();
  return 0;
}
