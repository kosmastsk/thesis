/* particle_filter.cpp */

#include "particle_filter/particle_filter.h"

namespace pf
{
/******************************/
/*        Constructor         */
/******************************/

Particles::Particles()
{
  // Get the parameters from Parameter Server
  _nh.param<int>("/particles", _numParticles, 500);

  _nh.param<std::string>("/worldFrame", _worldFrameID, "/world");
  _nh.param<std::string>("baseFootprintFrame", _baseFootprintFrameID, "/base_footprint");
  _nh.param<std::string>("baseStabilizedFrame", _baseStabilizedFrameID, "/base_stabilized");
  _nh.param<std::string>("baseLinkFrame", _baseLinkFrameID, "/base_link");

  // Initialize TF Listener/Broadcaster
  _tfBuffer.clear();
  _tfListener = new tf2_ros::TransformListener(_tfBuffer);
  _tfBroadcaster = new tf2_ros::TransformBroadcaster;

  // Initialize the Publishers
  _particlePublisher = _nh.advertise<geometry_msgs::PoseArray>("/particlecloud", 50);
  _posePublisher = _nh.advertise<geometry_msgs::Pose>("/amcl/pose", 50);

  // Initialize Models
  _mapModel = std::shared_ptr<MapModel>(new MapModel(&_nh));
  // octomap_server must have already provided the map to proceed

  _mm = new DroneMovementModel(&_nh, &_tfBuffer, _worldFrameID, _baseLinkFrameID);
  _om = std::shared_ptr<libPF::ObservationModel<DroneState> >(new DroneObservationModel(&_nh, _mapModel));

  _pf = new libPF::ParticleFilter<DroneState>(_numParticles, _om.get(), _mm);

  // Initialize the Subscribers
  _scanListener = _nh.subscribe("/scan", 10, &Particles::scanCallback, this);
  _odomListener = _nh.subscribe("/odom", 10, &Particles::odomCallback, this);

  _latestTransform.setIdentity();

  // Init PoseArray
  _poseArray.header.frame_id = _worldFrameID;
  _poseArray.poses.resize(_numParticles);

  // Crashes here, since pf in not initialized yet
  ROS_INFO("Particle filter created with %d particles!\n", _pf->numParticles());
}

/******************************/
/*         Destructor         */
/******************************/

Particles::~Particles()
{
  ROS_INFO("Particles object destroyed");
}

/******************************/
/*         initState          */
/******************************/

void Particles::initState()
{
  double x_pos, y_pos, z_pos, roll, pitch, yaw;

  // _nh.param<std::string>("default_param", default_param, "default_value");
  // Pose parameters
  _nh.param<double>("/x_pos", x_pos, 0);
  _nh.param<double>("/y_pos", y_pos, 0);
  _nh.param<double>("/z_pos", z_pos, 0);

  _nh.param<double>("/roll", roll, 0);
  _nh.param<double>("/pitch", pitch, 0);
  _nh.param<double>("/yaw", yaw, 0);

  // Initialize state variables
  _ds.setXPos(x_pos);
  _ds.setYPos(y_pos);
  _ds.setZPos(z_pos);

  _ds.setRoll(roll);
  _ds.setRoll(pitch);
  _ds.setYaw(yaw);

  // Initialize particle filter's state
  _pf->setPriorState(_ds);
}

/******************************/
/*       scanCallback         */
/******************************/

void Particles::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
}

/******************************/
/*       odomCallback         */
/******************************/

void Particles::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
}

}  // namespace pf
