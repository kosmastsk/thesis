#ifndef DRONESTATEDISTRIBUTION_H
#define DRONESTATEDISTRIBUTION_H

#include <iostream>
#include <memory>

#include <ros/ros.h>

#include <libPF/StateDistribution.h>
#include <libPF/CRandomNumberGenerator.h>

#include "particle_filter/DroneState.h"

namespace libPF
{
class RandomNumberGenerationStrategy;
}

class DroneStateDistribution : public libPF::StateDistribution<DroneState>
{
public:
  // Uniform distribution
  DroneStateDistribution(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, float rollmin,
                         float rollmax, float pitchmin, float pitchmax, float yawmin, float yawmax);
  // Gaussian Distribution
  DroneStateDistribution(float xInit, float yInit, float zInit, float rollInit, float pitchInit, float yawInit);

  // TODO add constructor for global localization with map shared ptr

  ~DroneStateDistribution();

  void setUniform(bool uniform);

  void setStdDev(float x, float y, float z, float r, float p, float yaw);

  const DroneState draw() const;

private:
  float _XMin, _XMax, _YMin, _YMax, _ZMin, _ZMax, _RollMin, _RollMax, _PitchMin, _PitchMax, _YawMin, _YawMax;
  float _XStdDev, _YStdDev, _ZStdDev, _RollStdDev, _PitchStdDev, _YawStdDev;
  bool _uniform;

  libPF::RandomNumberGenerationStrategy* m_RNG;
};

#endif  // DRONESTATEDISTRIBUTION_H
