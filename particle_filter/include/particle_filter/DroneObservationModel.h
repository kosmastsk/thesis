/*
* Copyright (c) 2019 Kosmas Tsiakas
*
* GNU GENERAL PUBLIC LICENSE
*    Version 3, 29 June 2007
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
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

  void setObservedMeasurements(pcl::PointCloud<pcl::PointXYZ> const& observed, std::vector<float> const& ranges);

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
