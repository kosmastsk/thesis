#ifndef DRONEMOVEMENTMODEL_H
#define DRONEMOVEMENTMODEL_H

#include <libPF/MovementModel.h>

#include "particle_filter/DroneState.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

class RandomNumberGenerator;

/**
 * @class DroneMovementModel
 *
 * @brief Test class for ParticleFilter.
 *
 * This movement model propagates a drone state according to the translation and
 * rotation speed.
 *
 * @author Stephan Wirth
 */
class DroneMovementModel : public libPF::MovementModel<DroneState>
{
public:
  /**
   * Constructor
   */
  DroneMovementModel(ros::NodeHandle* nh, tf2_ros::Buffer* tfBuffer, const std::string& worldFrameID,
                     const std::string& baseFrameID);

  /**
   * Destructor
   */
  ~DroneMovementModel();

  /**
   * The drift method propagates the car using its speed.
   * @param state Pointer to the state that has to be manipulated.
   */
  void drift(DroneState& state, double dt) const;

  /**
   * The diffusion consists of a very small gaussian jitter on the
   * state's variable.
   * @param state Pointer to the state that has to be manipulated.
   */
  void diffuse(DroneState& state, double dt) const;

  // param d new standard deviation for the diffusion of x
  void setXStdDev(double d);

  // return the standard deviation for the diffusion of x
  double getXStdDev() const;

  // param d new standard deviation for the diffusion of y
  void setYStdDev(double d);

  // return the standard deviation for the diffusion of y
  double getYStdDev() const;

  // param d new standard deviation for the diffusion of z
  void setZStdDev(double d);

  // return the standard deviation for the diffusion of z
  double getZStdDev() const;

  // param d new standard deviation for the diffusion of roll
  void setRollStdDev(double d);

  // return the standard deviation for the diffusion of roll
  double getRollStdDev() const;

  // param d new standard deviation for the diffusion of pitch
  void setPitchStdDev(double d);

  // return the standard deviation for the diffusion of pitch
  double getPitchStdDev() const;

  // param d new standard deviation for the diffusion of yaw
  void setYawStdDev(double d);

  // param return the standard deviation for the diffusion of yaw
  double getYawStdDev() const;

  // param odom new odometry pose
  void setLastOdomPose(geometry_msgs::PoseStamped& odomPose);

  /// get the last stored odomPose
  /// returns false when there is no valid previous pose stored
  bool getLastOdomPose(geometry_msgs::PoseStamped& lastOdomPose) const;

  // param currentPose, the current pose that the transform will be applied
  geometry_msgs::TransformStamped computeOdomTransform(geometry_msgs::PoseStamped& currentPose) const;

  // param odomTransform, the transform will be applied to statePose
  void applyOdomTransform(geometry_msgs::TransformStamped& odomTransform, geometry_msgs::Pose& statePose) const;

  /// look up the odom pose at a certain time through tf
  bool lookupOdomPose(const ros::Time& t, geometry_msgs::PoseStamped& pose) const;

  /// looks up the odometry pose at time t and then calls computeOdomTransform()
  bool lookupOdomTransform(const ros::Time& t, geometry_msgs::TransformStamped& odomTransform) const;

protected:
private:
  /// Stores the random number generator
  libPF::RandomNumberGenerationStrategy* m_RNG;

  bool _odometryReceived;

  std::string _worldFrameID;
  std::string _baseLinkFrameID;

  tf2_ros::Buffer* _tfBuffer;
  tf2_ros::TransformListener* _tfListener;

  geometry_msgs::PoseStamped _lastOdomPose;

  /// Store the standard deviations of the model
  double _XStdDev;
  double _YStdDev;
  double _ZStdDev;
  double _RollStdDev;
  double _PitchStdDev;
  double _YawStdDev;
};

#endif
