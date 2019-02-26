#ifndef DRONEOBSERVATIONMODEL_H
#define DRONEOBSERVATIONMODEL_H

#include <vector>
#include <libPF/ObservationModel.h>

#include "particle_filter/DroneState.h"

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
  DroneObservationModel();

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

  void setTrueDroneState(const DroneState& state);

protected:
private:
  DroneState m_TrueDroneState;
};

#endif
