#include <drone_gazebo/height_receiver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "estimator");

  height_receiver::Estimator object;

  ros::spin();

  return 0;
}
