#include <drone_coverage/coverage.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_coverage_node");

  drone_coverage::Coverage finder;

  ros::spin();
  return 0;
}
