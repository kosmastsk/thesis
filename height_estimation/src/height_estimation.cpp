#include <height_estimation.h>

namespace height_estimation
{

Estimator::Estimator()
{
  height_.data = 0.0;

  pub_ = nh_.advertise<std_msgs::Float64>("/height",1000);

  sub_ = nh_.subscribe("/lidar/height/scan", 1000, &Estimator::laserCallback, this);
}

Estimator::~Estimator()
{
  ROS_INFO("[Estimator] has been destroyed");
}

void Estimator::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    float height_avg = 0.0;
    int number_of_ranges = msg->ranges.size();
    float sum = 0.0; // temp variable
	  for (int i=0; i<number_of_ranges; i++)
        sum += msg->ranges[i];

    height_avg = sum / number_of_ranges;

    height_.data = height_avg;
    pub_.publish(height_); // Publish the height in the /height topic

    ROS_INFO("Current height: [%f]", height_avg);
}




}
