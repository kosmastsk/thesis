#ifndef DRONESTATEDISTRIBUTION_H
#define DRONESTATEDISTRIBUTION_H

#include <iostream>
#include <memory>

#include <ros/ros.h>

#include <libPF/StateDistribution.h>
#include <libPF/CRandomNumberGenerator.h>

#include "particle_filter/DroneState.h"
#include "particle_filter/MapModel.h"

namespace libPF
{
class RandomNumberGenerationStrategy;
}

class DroneStateDistribution : public libPF::StateDistribution<DroneState>
{
public:
  // Uniform distribution
  DroneStateDistribution(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, double rollmin,
                         double rollmax, double pitchmin, double pitchmax, double yawmin, double yawmax);
  // Gaussian Distribution
  DroneStateDistribution(double xInit, double yInit, double zInit, double rollInit, double pitchInit, double yawInit);

  // Global localization
  DroneStateDistribution(std::shared_ptr<MapModel> map);

  ~DroneStateDistribution();

  void setUniform(bool uniform);

  void setStdDev(double x, double y, double z, double r, double p, double yaw);

  const DroneState draw() const;

private:
  double _XMin, _XMax, _YMin, _YMax, _ZMin, _ZMax, _RollMin, _RollMax, _PitchMin, _PitchMax, _YawMin, _YawMax;
  double _XStdDev, _YStdDev, _ZStdDev, _RollStdDev, _PitchStdDev, _YawStdDev;
  bool _uniform;
  std::shared_ptr<octomap::ColorOcTree> _map;

  libPF::RandomNumberGenerationStrategy* m_RNG;
};

#endif  // DRONESTATEDISTRIBUTION_H
