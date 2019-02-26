#include <cmath>

#include "particle_filter/DroneState.h"

DroneState::DroneState() : x_pos(0.0), y_pos(0.0), z_pos(0.0), roll(0.0), pitch(0.0), yaw(0.0)
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

DroneState DroneState::operator*(float factor) const
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

float DroneState::getXPos() const
{
  return x_pos;
}

void DroneState::setXPos(float x)
{
  x_pos = x;
}

float DroneState::getYPos() const
{
  return y_pos;
}

void DroneState::setYPos(float y)
{
  y_pos = y;
}

float DroneState::getZPos() const
{
  return z_pos;
}

void DroneState::setZPos(float z)
{
  z_pos = z;
}

float DroneState::getRoll() const
{
  return roll;
}

void DroneState::setRoll(float r)
{
  roll = r;
}

float DroneState::getPitch() const
{
  return pitch;
}

void DroneState::setPitch(float p)
{
  pitch = p;
}

float DroneState::getYaw() const
{
  return yaw;
}

void DroneState::setYaw(float y)
{
  yaw = y;
}
