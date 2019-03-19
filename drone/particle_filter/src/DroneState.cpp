#include <cmath>

#include "particle_filter/DroneState.h"

DroneState::DroneState()
{
}

DroneState::~DroneState()
{
}

DroneState& DroneState::operator=(const DroneState& other)
{
  x_pos = other.x_pos;
  y_pos = other.y_pos;
  z_pos = other.z_pos;

  roll = other.roll;
  pitch = other.pitch;
  yaw = other.yaw;

  return *this;
}

DroneState& DroneState::operator+=(const DroneState& other)
{
  x_pos += other.x_pos;
  y_pos += other.y_pos;
  z_pos += other.z_pos;

  roll += other.roll;
  pitch += other.pitch;
  yaw += other.yaw;

  return *this;
}

DroneState DroneState::operator*(double factor) const
{
  DroneState newState;
  newState.x_pos = x_pos * factor;
  newState.y_pos = y_pos * factor;
  newState.z_pos = z_pos * factor;

  newState.roll = roll * factor;
  newState.pitch = pitch * factor;
  newState.yaw = yaw * factor;

  return newState;
}

double DroneState::getXPos() const
{
  return x_pos;
}

void DroneState::setXPos(double x)
{
  x_pos = x;
}

double DroneState::getYPos() const
{
  return y_pos;
}

void DroneState::setYPos(double y)
{
  y_pos = y;
}

double DroneState::getZPos() const
{
  return z_pos;
}

void DroneState::setZPos(double z)
{
  z_pos = z;
}

double DroneState::getRoll() const
{
  return roll;
}

void DroneState::setRoll(double r)
{
  roll = r;
}

double DroneState::getPitch() const
{
  return pitch;
}

void DroneState::setPitch(double p)
{
  pitch = p;
}

double DroneState::getYaw() const
{
  return yaw;
}

void DroneState::setYaw(double y)
{
  yaw = y;
}
