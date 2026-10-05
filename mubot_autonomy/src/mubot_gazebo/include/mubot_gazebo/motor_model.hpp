// Actuator model shared by the Gazebo system and its unit tests.
// Port of the torque law in legacy actuator_plugin.cc.
#pragma once

#include <algorithm>
#include <cmath>

namespace mubot_gazebo
{

struct MotorParams
{
  double torque_constant = 0.00126;  // k [N·m/A], also back-EMF constant
  double resistance = 90.0;          // R [ohm]
  double voltage_limit = 15.0;       // [V]
  double ac_ratio = 1.0;             // legacy scale factor
};

// Gait waveform for one active joint: e·sin(2π(f·t + ψ)) + b, before clipping.
inline double waveform(double amplitude, double frequency, double phase, double t, double bias)
{
  return amplitude * std::sin(2.0 * M_PI * (frequency * t + phase)) + bias;
}

inline double clip_voltage(double v, const MotorParams & p, bool * clipped = nullptr)
{
  const double out = std::clamp(v, -p.voltage_limit, p.voltage_limit);
  if (clipped) {*clipped = (out != v);}
  return out;
}

// Motor torque: (E − k·ω)/R · k.
inline double motor_torque(double voltage, double joint_velocity, const MotorParams & p)
{
  return (voltage - p.torque_constant * joint_velocity) / p.resistance * p.torque_constant *
         p.ac_ratio;
}

inline double spring_torque(double stiffness, double position, const MotorParams & p)
{
  return -stiffness * position * p.ac_ratio;
}

// Electrical power drawn by one actuator.
inline double electrical_power(double voltage, double joint_velocity, const MotorParams & p)
{
  const double current = (voltage - p.torque_constant * joint_velocity) / p.resistance;
  return voltage * current;
}

}  // namespace mubot_gazebo
