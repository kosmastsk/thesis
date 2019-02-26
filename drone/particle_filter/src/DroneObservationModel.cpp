#include "particle_filter/DroneObservationModel.h"
#include "particle_filter/MapModel.h"

DroneObservationModel::DroneObservationModel() : libPF::ObservationModel<DroneState>()
{
}

DroneObservationModel::~DroneObservationModel()
{
}

double DroneObservationModel::measure(const DroneState& state) const
{
  /*double xdist = state.getPose().position.x - m_TrueDroneState.getPose().position.x;
  double ydist = state.getPose().position.y - m_TrueDroneState.getPose().position.y;
  double zdist = state.getPose().position.z - m_TrueDroneState.getPose().position.z;
  double dist3 = (xdist * xdist + ydist * ydist + zdist * zdist);
  if (dist3 < 0.5)
    dist3 = 0.5;  // avoid division by zero
  return 1.0 / dist3;*/
}

void DroneObservationModel::setTrueDroneState(const DroneState& state)
{
  m_TrueDroneState = state;
}
