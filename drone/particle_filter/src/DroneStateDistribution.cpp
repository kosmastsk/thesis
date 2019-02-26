#include <cmath>
#include <libPF/CRandomNumberGenerator.h>

#include "particle_filter/DroneStateDistribution.h"

DroneStateDistribution::DroneStateDistribution(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
  : _XMin(xmin), _XMax(xmax), _YMin(ymin), _YMax(ymax), _ZMin(zmin), _ZMax(zmax)
{
  m_RNG = new libPF::CRandomNumberGenerator();
}

DroneStateDistribution::~DroneStateDistribution()
{
  delete m_RNG;
}

const DroneState DroneStateDistribution::draw() const
{
  DroneState state;
  /*  state.setXPos(m_RNG->getUniform(_XMin, _XMax));
    state.setYPos(m_RNG->getUniform(_YMin, _YMax));
    state.setZPos(m_RNG->getUniform(_ZMin, _ZMax));
    state.setTheta(m_RNG->getUniform(-M_PI, M_PI));*/
  return state;
}
