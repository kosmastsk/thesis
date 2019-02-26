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

class DroneState
{
public:
  DroneState();
  ~DroneState();

  DroneState& operator=(const DroneState& other);
  DroneState& operator+=(const DroneState& other);
  DroneState operator*(float factor) const;

  float getXPos() const;
  void setXPos(float x);

  float getYPos() const;
  void setYPos(float y);

  float getZPos() const;
  void setZPos(float z);

  float getRoll() const;
  void setRoll(float r);

  float getPitch() const;
  void setPitch(float p);

  float getYaw() const;
  void setYaw(float y);

private:
  float x_pos;
  float y_pos;
  float z_pos;
  float roll;
  float pitch;
  float yaw;
};

#endif  // DRONESTATE_H
