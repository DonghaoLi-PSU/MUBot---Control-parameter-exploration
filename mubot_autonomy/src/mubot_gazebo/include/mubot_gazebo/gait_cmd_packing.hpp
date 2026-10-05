// Packing of GaitCmd / ActuatorState into a flat double array, the payload of
// gz.msgs.Double_V on the Gazebo side. Shared by the actuator system and the
// ROS ↔ gz command bridge.
//
// GaitCmd:       [n, e_1..e_n, psi_1..psi_n, f, bias, stiffness, tail_ratio, switch_at_phase]
// ActuatorState: [n, v_1..v_n, clipped_1..clipped_n, power, watchdog_tripped, estop]
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace mubot_gazebo
{

struct GaitCommand
{
  std::vector<double> amplitude;
  std::vector<double> phase;
  double frequency = 0.0;
  double bias = 0.0;
  double stiffness = 0.0;
  double tail_ratio = 5.0;
  bool switch_at_phase = false;
};

inline std::vector<double> pack(const GaitCommand & c)
{
  const std::size_t n = c.amplitude.size();
  std::vector<double> d;
  d.reserve(2 * n + 6);
  d.push_back(static_cast<double>(n));
  d.insert(d.end(), c.amplitude.begin(), c.amplitude.end());
  d.insert(d.end(), c.phase.begin(), c.phase.end());
  d.push_back(c.frequency);
  d.push_back(c.bias);
  d.push_back(c.stiffness);
  d.push_back(c.tail_ratio);
  d.push_back(c.switch_at_phase ? 1.0 : 0.0);
  return d;
}

inline std::optional<GaitCommand> unpack_gait(const std::vector<double> & d)
{
  if (d.empty()) {return std::nullopt;}
  const auto n = static_cast<std::size_t>(d[0]);
  if (d.size() != 2 * n + 6) {return std::nullopt;}
  GaitCommand c;
  c.amplitude.assign(d.begin() + 1, d.begin() + 1 + n);
  c.phase.assign(d.begin() + 1 + n, d.begin() + 1 + 2 * n);
  c.frequency = d[2 * n + 1];
  c.bias = d[2 * n + 2];
  c.stiffness = d[2 * n + 3];
  c.tail_ratio = d[2 * n + 4];
  c.switch_at_phase = d[2 * n + 5] > 0.5;
  return c;
}

struct ActuatorStateData
{
  std::vector<double> voltage;
  std::vector<bool> clipped;
  double power = 0.0;
  bool watchdog_tripped = false;
  bool estop = false;
};

inline std::vector<double> pack(const ActuatorStateData & s)
{
  std::vector<double> d{static_cast<double>(s.voltage.size())};
  d.insert(d.end(), s.voltage.begin(), s.voltage.end());
  for (bool b : s.clipped) {d.push_back(b ? 1.0 : 0.0);}
  d.push_back(s.power);
  d.push_back(s.watchdog_tripped ? 1.0 : 0.0);
  d.push_back(s.estop ? 1.0 : 0.0);
  return d;
}

inline std::optional<ActuatorStateData> unpack_state(const std::vector<double> & d)
{
  if (d.empty()) {return std::nullopt;}
  const auto n = static_cast<std::size_t>(d[0]);
  if (d.size() != 2 * n + 4) {return std::nullopt;}
  ActuatorStateData s;
  s.voltage.assign(d.begin() + 1, d.begin() + 1 + n);
  for (std::size_t i = 0; i < n; ++i) {s.clipped.push_back(d[1 + n + i] > 0.5);}
  s.power = d[2 * n + 1];
  s.watchdog_tripped = d[2 * n + 2] > 0.5;
  s.estop = d[2 * n + 3] > 0.5;
  return s;
}

}  // namespace mubot_gazebo
