// Per-segment hydrodynamic force model, ported line for line from legacy
// Hydro_plugin.cc (reactive added-mass forces + head/tail pressure force +
// resistive drag and friction). Pure math, no Gazebo types, so it can be
// unit-tested and compared against the legacy plugin.
#pragma once

#include <array>
#include <cmath>

namespace mubot_gazebo
{

enum SegmentType { HEAD = 0, BODY = 1, TAIL = 2, TIP = 3 };

struct SegmentHydro
{
  double M_total, M_S, cog, lt, IH0, IH1, IH2, IH3, IP;
};

struct HydroParams
{
  // Model coefficients (HM1..HM4)
  double Ca = 1.0;               // legacy Seg_Cm
  double Cp = 1.0;               // legacy loco_indicator
  double rho = 1000.0;           // legacy pho_f
  double M_total_ratio = 0.75;   // legacy Seg_M_total_ratio
  // Body coefficients (AR*)
  double c_d = 1.2;
  double c_f = 0.006;
  double M_0 = 0.0;
  double M_l = 0.0;
  double h_0 = 0.0;
  std::array<SegmentHydro, 4> seg{};  // head, body, tail, tip
};

// Kinematics of one link, as Gazebo reports them in the world frame.
struct SegmentKinematics
{
  double yaw;        // link yaw [rad]
  double vx, vy;     // linear velocity of the link origin (front joint), world [m/s]
  double ax, ay;     // linear acceleration, world [m/s^2]
  double wz;         // yaw rate [rad/s]
  double alpha_z;    // yaw acceleration [rad/s^2]
};

// Wrench to apply: force in the link frame, torque about world z.
struct SegmentWrench
{
  double fx = 0.0, fy = 0.0, tz = 0.0;
  double pressure_x = 0.0;  // the Cp term alone, for logging
};

// Segment index i of s_num segments → parameter table used by the legacy code.
inline SegmentType segment_type(int i, int s_num)
{
  if (i == 0) {return HEAD;}
  if (i == s_num - 1) {return TIP;}
  if (i == s_num - 2) {return TAIL;}
  return BODY;
}

inline double sgn(double x) {return (x > 0.0) - (x < 0.0);}

inline SegmentWrench compute_wrench(const HydroParams & p, SegmentType type, const SegmentKinematics & k)
{
  const SegmentHydro & s = p.seg[type];
  const double c = std::cos(k.yaw), sn = std::sin(k.yaw);

  // World → mobile (link) frame.
  const double vel_x = k.vx * c + k.vy * sn;
  const double vel_y = k.vy * c - k.vx * sn;
  const double acc_x = k.ax * c + k.ay * sn;
  const double acc_y = -k.ax * sn + k.ay * c;
  const double w = k.wz;
  const double vel_y_l = vel_y + w * s.lt;          // lateral velocity at the rear point
  const double acc_y_ant = acc_y - s.cog * k.alpha_z;

  SegmentWrench out;

  // Reactive (added-mass) forces.
  double react_x = p.Ca * (s.M_S * w * w + s.M_total * vel_y * w) +
    s.M_total * p.M_total_ratio * acc_x;
  double react_y = p.Ca * (s.M_total * vel_x * w - s.M_total * acc_y_ant - s.M_S * k.alpha_z -
    p.M_l * vel_x * vel_y_l + p.M_0 * vel_x * vel_y) +
    s.M_total * p.M_total_ratio * acc_y;
  double react_zz = -p.Ca * (p.M_l * s.lt * vel_x * vel_y_l + s.M_S * acc_y_ant -
    s.M_S * vel_x * w) +
    s.M_total * p.M_total_ratio * acc_y * s.cog;

  // Pressure force at the head and at the tip (scaled by Cp).
  if (type == HEAD) {
    out.pressure_x = 0.5 * p.Cp * p.M_0 * vel_y * vel_y;
  } else if (type == TIP) {
    out.pressure_x = -0.5 * p.Cp * p.M_l * vel_y_l * vel_y_l;
  }
  react_x += out.pressure_x;

  // Resistive forces.
  const double eps = 1e-8;
  const double k_d = -0.5 * p.rho * p.c_d;
  const double resis_x = (-0.5 * p.rho * p.c_f) * std::fabs(vel_x) * vel_x * s.IP;
  double resis_y = 0.0, resis_t = 0.0;
  const double lc = (std::fabs(w) >= eps) ? -vel_y / w : 0.0;  // zero-velocity point

  if (std::fabs(w) <= eps) {
    resis_y = k_d * std::fabs(vel_y) * vel_y * s.IH0;
    resis_t = k_d * std::fabs(vel_y) * vel_y * s.IH1;
  } else if (std::fabs(vel_y) <= eps) {
    resis_y = k_d * std::fabs(w) * w * s.IH2;
    resis_t = k_d * std::fabs(w) * w * s.IH3;
  } else if (lc < 0.0 || lc >= s.lt) {
    // Whole segment moves sideways in one direction.
    resis_y = k_d * sgn(vel_y) * (vel_y * vel_y * s.IH0 + 2.0 * w * vel_y * s.IH1 + w * w * s.IH2);
    resis_t = k_d * sgn(vel_y) * (vel_y * vel_y * s.IH1 + 2.0 * w * vel_y * s.IH2 + w * w * s.IH3);
  } else {
    // Zero-velocity point lies on the segment: integrate both sides.
    const double lt = s.lt;
    resis_y = k_d * p.h_0 * sgn(vel_y) *
      (vel_y * vel_y * (2.0 * lc - lt) +
      vel_y * w * (2.0 * lc * lc - lt * lt) +
      w * w * (2.0 / 3.0 * std::pow(lc, 3) - 1.0 / 3.0 * std::pow(lt, 3)));
    resis_t = k_d * p.h_0 * sgn(vel_y) *
      (vel_y * vel_y * (lc * lc - 0.5 * lt * lt) +
      vel_y * w * (4.0 / 3.0 * std::pow(lc, 3) - 2.0 / 3.0 * std::pow(lt, 3)) +
      w * w * (0.5 * std::pow(lc, 4) - 0.25 * std::pow(lt, 4)));
  }

  out.fx = react_x + resis_x;
  out.fy = react_y + resis_y;
  out.tz = react_zz + resis_t;
  return out;
}

}  // namespace mubot_gazebo
