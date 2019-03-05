#include <cmath>
#include <libPF/CRandomNumberGenerator.h>

#include "particle_filter/DroneStateDistribution.h"

// Uniform
DroneStateDistribution::DroneStateDistribution(double xmin, double xmax, double ymin, double ymax, double zmin,
                                               double zmax, double rollmin, double rollmax, double pitchmin,
                                               double pitchmax, double yawmin, double yawmax)
  : _XMin(xmin)
  , _XMax(xmax)
  , _YMin(ymin)
  , _YMax(ymax)
  , _ZMin(zmin)
  , _ZMax(zmax)
  , _RollMin(rollmin)
  , _RollMax(rollmax)
  , _PitchMin(pitchmin)
  , _PitchMax(pitchmax)
  , _YawMin(yawmin)
  , _YawMax(yawmax)
{
  m_RNG = new libPF::CRandomNumberGenerator();
  _uniform = true;
}

// Gauss
DroneStateDistribution::DroneStateDistribution(double xInit, double yInit, double zInit, double rollInit,
                                               double pitchInit, double yawInit)
{
  _XMin = xInit;
  _YMin = yInit;
  _ZMin = zInit;
  _RollMin = rollInit;
  _PitchMin = pitchInit;
  _YawMin = yawInit;
  m_RNG = new libPF::CRandomNumberGenerator();
  _uniform = false;
}

DroneStateDistribution::DroneStateDistribution(std::shared_ptr<MapModel> map)
{
  _map = map->getMap();
  m_RNG = new libPF::CRandomNumberGenerator();
  double zmin, zmax;
  _map->getMetricMin(_XMin, _YMin, _ZMin);
  _map->getMetricMax(_XMax, _YMax, _ZMax);
  _RollMin = -M_PI;
  _PitchMin = -M_PI;
  _YawMin = -M_PI;
  _RollMax = M_PI;
  _PitchMax = M_PI;
  _YawMax = M_PI;
  _uniform = true;
}

DroneStateDistribution::~DroneStateDistribution()
{
  delete m_RNG;
}

void DroneStateDistribution::setUniform(bool uniform)
{
  _uniform = uniform;
}

void DroneStateDistribution::setStdDev(double x, double y, double z, double r, double p, double yaw)
{
  _XStdDev = x;
  _YStdDev = y;
  _ZStdDev = z;
  _RollStdDev = r;
  _PitchStdDev = p;
  _YawStdDev = yaw;
}

const DroneState DroneStateDistribution::DroneStateDistribution::draw() const
{
  DroneState state;
  if (_uniform)
  {
    /**
     * Generates a uniform distributed random number between min and max.
     * @param min the minimum value, default is 0.0
     * @param max the maximum value, default is 1.0
     * @return random number between min and max, uniform distributed.
     */
    state.setXPos(m_RNG->getUniform(_XMin, _XMax));
    state.setYPos(m_RNG->getUniform(_YMin, _YMax));
    state.setZPos(m_RNG->getUniform(_ZMin, _ZMax));
    state.setRoll(m_RNG->getUniform(_RollMin, _RollMax));
    state.setPitch(m_RNG->getUniform(_PitchMin, _PitchMax));
    state.setYaw(m_RNG->getUniform(_YawMin, _YawMax));
  }
  else
  {
    /**
    * This method creates gaussian distributed random numbers (Box-Müller method).
    * @param standardDeviation Standard deviation d of the random number to generate.
    * @return N(0, d*d)-distributed random number
    */
    state.setXPos(m_RNG->getGaussian(_XStdDev));
    state.setYPos(m_RNG->getGaussian(_YStdDev));
    state.setZPos(m_RNG->getGaussian(_ZStdDev));
    state.setRoll(m_RNG->getGaussian(_RollStdDev));
    state.setPitch(m_RNG->getGaussian(_PitchStdDev));
    state.setYaw(m_RNG->getGaussian(_YawStdDev));
  }

  return state;
}
