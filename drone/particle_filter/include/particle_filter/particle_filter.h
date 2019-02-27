#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

// System headers
#include <iostream>
#include <iomanip>

// ROS headers
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

// ROS messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

// libPF headers
#include <libPF/ParticleFilter.h>

// Project headers
#include "particle_filter/DroneObservationModel.h"
#include "particle_filter/DroneMovementModel.h"
#include "particle_filter/DroneStateDistribution.h"
#include "particle_filter/MapModel.h"

namespace pf
{
class Particles
{
protected:
  // Variables
  ros::NodeHandle _nh;
  DroneObservationModel _om;
  DroneMovementModel _mm;
  MapModel _mapModel;
  libPF::ParticleFilter<DroneState> _pf;
  DroneState _ds;
  int _particles;

  // Pub - Sub
  ros::Subscriber _scanListener;
  ros::Subscriber _odomListener;
  ros::Publisher _particlePublisher;
  ros::Publisher _posePublisher;

  // Frames
  std::string _worldFrameID;
  std::string _baseFootprintFrameID;
  std::string _baseStabilizedFrameID;
  std::string _baseLinkFrameID;

  // TFs
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  tf2_ros::TransformBroadcaster _tfBroadcaster;

  // Functions
  void initState();

  // Callbacks
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
  Particles();
  ~Particles();
};

}  // namespace pf

#endif  // PARTICLE_FILTER_H
