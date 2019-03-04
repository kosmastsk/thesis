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
#include <tf2_ros/message_filter.h>
#include <tf2/utils.h>

// Message Filters
#include <message_filters/subscriber.h>

// ROS messages
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// libPF headers
#include <libPF/ParticleFilter.h>

// PCL PointCloud
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/keypoints/uniform_sampling.h>

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
  // Models related
  ros::NodeHandle _nh;
  std::shared_ptr<libPF::ObservationModel<DroneState> > _om;
  DroneMovementModel* _mm;
  std::shared_ptr<MapModel> _mapModel;
  libPF::ParticleFilter<DroneState>* _pf;
  DroneState _ds;
  int _numParticles;

  // Pub - Sub
  message_filters::Subscriber<sensor_msgs::LaserScan>* _scanListener;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan>* _scanFilter;

  ros::Subscriber _odomListener;

  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* _initialPoseListener;
  tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* _initialPoseFilter;

  ros::Publisher _particlePublisher;
  ros::Publisher _posePublisher;
  ros::Publisher _poseArrayPublisher;
  ros::Publisher _filteredPointCloudPublisher;

  // Frames
  std::string _mapFrameID;
  std::string _worldFrameID;
  std::string _baseFootprintFrameID;
  std::string _baseStabilizedFrameID;
  std::string _baseLinkFrameID;

  // TFs
  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener* _tfListener;
  tf2_ros::TransformBroadcaster* _tfBroadcaster;
  tf2::Transform _latestTransform;

  // Pose
  geometry_msgs::PoseArray _poseArray;
  geometry_msgs::Pose _lastLocalizedPose;

  // Useful variables
  bool _initialized;
  bool _firstRun;
  bool _receivedSensorData;
  bool _publishUpdated;

  // Laser
  double _filterMinRange;
  double _filterMaxRange;
  double _observationThresholdTranslation;
  double _observationThresholdRotation;
  double _sensorSampleDist;
  double _transformTolerance;

  // Particles standard deviation
  double _XStdDev, _YStdDev, _ZStdDev, _RollStdDev, _PitchStdDev, _YawStdDev;

  ros::Time _lastLaserTime;
  ros::Timer _latestTransformTimer;

  // Functions
  void initState();

  void publishPoseEstimate(const ros::Time& t);
  void prepareLaserPointCloud(const sensor_msgs::LaserScanConstPtr& scan, pcl::PointCloud<pcl::PointXYZ>& pc,
                              std::vector<float>& ranges) const;

  bool isAboveMotionThreshold(const geometry_msgs::PoseStamped& odomPose) const;

  // Callbacks
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void latestTransformTimerCallback(const ros::TimerEvent& timer_event);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

public:
  Particles();
  ~Particles();
};

}  // namespace pf

#endif  // PARTICLE_FILTER_H
