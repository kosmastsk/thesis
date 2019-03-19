#ifndef DRONESTATE_H
#define DRONESTATE_H

/**
 * @class DroneState
 * @brief Sample state for a particle filter that simulates a drone.
 *
 * This state has the following parameters:
 * @li <b>xpos</b> the x-Position of the drone
 * @li <b>ypos</b> the y-Position of the drone
 * @li <b>zpos</b> the z-Position of the drone
 * @li <b>roll</b> the roll-Position of the drone (in radiants)
 * @li <b>pitch</b> the pitch-Position of the drone (in radiants)
 * @li <b>yaw</b> the yaw-Position of the drone
 * @li <b>velocity</b> the speed with which the drone moves, linear and angular (in m/s)
 */

#include <ros/ros.h>

class DroneState
{
public:
  DroneState();
  ~DroneState();

  DroneState& operator=(const DroneState& other);
  DroneState& operator+=(const DroneState& other);
  DroneState operator*(double factor) const;

  double getXPos() const;
  void setXPos(double x);

  double getYPos() const;
  void setYPos(double y);

  double getZPos() const;
  void setZPos(double z);

  double getRoll() const;
  void setRoll(double r);

  double getPitch() const;
  void setPitch(double p);

  double getYaw() const;
  void setYaw(double y);

private:
  double x_pos;
  double y_pos;
  double z_pos;
  double roll;
  double pitch;
  double yaw;
};

#endif  // DRONESTATE_H
