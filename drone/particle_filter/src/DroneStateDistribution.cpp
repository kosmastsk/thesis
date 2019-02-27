#include <cmath>
#include <libPF/CRandomNumberGenerator.h>

#include "particle_filter/DroneStateDistribution.h"

// Uniform
DroneStateDistribution::DroneStateDistribution(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax,
                                               float rollmin, float rollmax, float pitchmin, float pitchmax,
                                               float yawmin, float yawmax)
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
DroneStateDistribution::DroneStateDistribution(float xInit, float yInit, float zInit, float rollInit, float pitchInit,
                                               float yawInit)
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

DroneStateDistribution::~DroneStateDistribution()
{
  delete m_RNG;
}

void DroneStateDistribution::setUniform(bool uniform)
{
  _uniform = uniform;
}

void DroneStateDistribution::setStdDev(float x, float y, float z, float r, float p, float yaw)
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
