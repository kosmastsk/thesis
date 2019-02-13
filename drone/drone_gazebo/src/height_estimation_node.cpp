#include <drone_gazebo/height_estimation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  height_estimation::Estimator object;

  ros::spin();

  return 0;
}
