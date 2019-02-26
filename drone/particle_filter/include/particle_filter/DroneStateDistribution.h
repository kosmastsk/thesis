#ifndef DRONESTATEDISTRIBUTION_H
#define DRONESTATEDISTRIBUTION_H

#include <libPF/StateDistribution.h>

#include "particle_filter/DroneState.h"

namespace libPF
{
class RandomNumberGenerationStrategy;
}

class DroneStateDistribution : public libPF::StateDistribution<DroneState>
{
public:
  DroneStateDistribution(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);
  ~DroneStateDistribution();

  const DroneState draw() const;

private:
  float _XMin, _XMax, _YMin, _YMax, _ZMin, _ZMax;

  libPF::RandomNumberGenerationStrategy* m_RNG;
};

#endif  // DRONESTATEDISTRIBUTION_H
