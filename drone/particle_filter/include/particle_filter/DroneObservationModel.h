#ifndef DRONEOBSERVATIONMODEL_H
#define DRONEOBSERVATIONMODEL_H

#include <vector>
#include <libPF/ObservationModel.h>

#include "particle_filter/DroneState.h"
#include <particle_filter/MapModel.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

/**
 * @class DroneObservationModel
 *
 * @brief Observation model that measures difference between laser measurements and the estimated measurements of a
 * drone.
 *
 * @author Stephan Wirth
 *
 * @brief Test class for ParticleFilter.
 *
 */
class DroneObservationModel : public libPF::ObservationModel<DroneState>
{
public:
  /**
   * empty
   */
  DroneObservationModel(ros::NodeHandle* nh, std::shared_ptr<MapModel> _mapModel);

  /**
   * empty
   */
  ~DroneObservationModel();

  /**
   *
   * @param state Reference to the state that has to be weightened.
   * @return weight for the given state.
   */
  double measure(const DroneState& state) const;

  void setMap(const std::shared_ptr<octomap::ColorOcTree>& map);

  void setBaseToSensorTransform(const tf2::Transform& baseToSensorTF);

  void setObservedRanges(const pcl::PointCloud<pcl::PointXYZ>& observed, const std::vector<float>& ranges);

protected:
private:
  std::shared_ptr<octomap::ColorOcTree> _map;
  tf2::Transform _baseToSensorTransform;
  std::vector<float> _observedRanges;
  pcl::PointCloud<pcl::PointXYZ> _observedMeasurement;

  double _ZHit;
  double _ZShort;
  double _ZRand;
  double _ZMax;
  double _SigmaHit;
  double _LambdaShort;
  double _minRange;
  double _maxRange;
};

#endif
