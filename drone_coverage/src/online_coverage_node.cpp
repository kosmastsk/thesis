#include <drone_coverage/online_coverage.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_coverage_node");

  drone_coverage::OnlineCoverage online_coverage;

  ros::spin();
  return 0;
}
