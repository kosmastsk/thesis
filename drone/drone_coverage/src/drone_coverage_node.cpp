#include <drone_coverage/coverage_finder.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_coverage_node");

  drone_coverage::CoverageFinder finder;

  ros::spin();
  return 0;
}
