#include <gtest/gtest.h>

#include <cmath>

#include "mubot_gazebo/gait_cmd_packing.hpp"
#include "mubot_gazebo/hydro_model.hpp"
#include "mubot_gazebo/motor_model.hpp"

using namespace mubot_gazebo;

namespace
{
HydroParams ar2_hm3()
{
  HydroParams p;
  p.Ca = 1.0; p.Cp = 0.5; p.c_d = 1.2; p.c_f = 0.006;
  p.M_0 = 1.479e-1; p.M_l = 1.479e-1; p.h_0 = 13.72e-3;
  p.seg[HEAD] = {6.360e-3, 1.367e-4, 21.75e-3, 43.00e-3, 5.901e-4, 1.268e-5, 3.637e-7, 1.173e-8, 1.437e-3};
  p.seg[BODY] = {3.957e-3, 5.292e-5, 13.38e-3, 26.75e-3, 3.671e-4, 4.910e-6, 8.756e-8, 1.757e-9, 8.937e-4};
  p.seg[TAIL] = {2.921e-3, 2.885e-5, 7.934e-3, 19.75e-3, 2.710e-4, 2.676e-6, 3.524e-8, 5.220e-10, 4.965e-4};
  p.seg[TIP] = {4.141e-3, 5.798e-5, 14.00e-3, 28.00e-3, 3.842e-4, 5.379e-6, 1.004e-7, 2.109e-9, 8.231e-4};
  return p;
}
}  // namespace

TEST(Motor, ClipsAtFifteenVolts)
{
  MotorParams p;
  bool clipped = false;
  EXPECT_DOUBLE_EQ(clip_voltage(20.0, p, &clipped), 15.0);
  EXPECT_TRUE(clipped);
  EXPECT_DOUBLE_EQ(clip_voltage(-3.0, p, &clipped), -3.0);
  EXPECT_FALSE(clipped);
}

TEST(Motor, MatchesLegacyTorqueLaw)
{
  MotorParams p;
  // legacy: (voltage - 0.00126*vel)/90*0.00126
  EXPECT_NEAR(motor_torque(10.0, 2.0, p), (10.0 - 0.00126 * 2.0) / 90.0 * 0.00126, 1e-15);
  EXPECT_NEAR(spring_torque(3.75e-3, 0.1, p), -3.75e-4, 1e-15);
}

TEST(Motor, BiasShiftsWaveform)
{
  EXPECT_NEAR(waveform(10.0, 2.0, 0.0, 0.0, 3.0), 3.0, 1e-12);
  EXPECT_NEAR(waveform(10.0, 1.0, 0.0, 0.25, 0.0), 10.0, 1e-12);
}

TEST(Hydro, NoMotionNoForce)
{
  const auto p = ar2_hm3();
  for (int t = HEAD; t <= TIP; ++t) {
    const auto w = compute_wrench(p, static_cast<SegmentType>(t), {0.3, 0, 0, 0, 0, 0, 0});
    EXPECT_DOUBLE_EQ(w.fx, 0.0);
    EXPECT_DOUBLE_EQ(w.fy, 0.0);
    EXPECT_DOUBLE_EQ(w.tz, 0.0);
  }
}

TEST(Hydro, SurgeFrictionOpposesMotion)
{
  const auto p = ar2_hm3();
  const auto w = compute_wrench(p, BODY, {0.0, 0.1, 0, 0, 0, 0, 0});
  EXPECT_LT(w.fx, 0.0);
  EXPECT_NEAR(w.fx, -0.5 * 1000 * 0.006 * 0.1 * 0.1 * 8.937e-4, 1e-15);
}

TEST(Hydro, MirrorSymmetry)
{
  const auto p = ar2_hm3();
  const auto a = compute_wrench(p, BODY, {0.0, -0.05, 0.02, 0.1, 0.3, 1.5, 4.0});
  const auto b = compute_wrench(p, BODY, {0.0, -0.05, -0.02, 0.1, -0.3, -1.5, -4.0});
  EXPECT_NEAR(a.fx, b.fx, 1e-12);
  EXPECT_NEAR(a.fy, -b.fy, 1e-12);
  EXPECT_NEAR(a.tz, -b.tz, 1e-12);
}

TEST(Hydro, PressureScalesWithCp)
{
  auto p = ar2_hm3();
  const SegmentKinematics k{0.0, -0.05, 0.03, 0, 0, 0, 0};
  const double half = compute_wrench(p, HEAD, k).pressure_x;
  p.Cp = 1.0;
  EXPECT_NEAR(compute_wrench(p, HEAD, k).pressure_x, 2.0 * half, 1e-15);
  p.Cp = 0.0;
  EXPECT_DOUBLE_EQ(compute_wrench(p, HEAD, k).pressure_x, 0.0);
  EXPECT_DOUBLE_EQ(compute_wrench(p, BODY, k).pressure_x, 0.0);
}

TEST(Hydro, SegmentTypes)
{
  EXPECT_EQ(segment_type(0, 6), HEAD);
  EXPECT_EQ(segment_type(1, 6), BODY);
  EXPECT_EQ(segment_type(3, 6), BODY);
  EXPECT_EQ(segment_type(4, 6), TAIL);
  EXPECT_EQ(segment_type(5, 6), TIP);
}

TEST(Packing, GaitRoundTrip)
{
  GaitCommand c{{10, 11, 12, 13}, {0, 0.2, 0.4, 0.6}, 3.0, -2.5, 3.75e-3, 5.0, true};
  const auto u = unpack_gait(pack(c));
  ASSERT_TRUE(u);
  EXPECT_EQ(u->amplitude, c.amplitude);
  EXPECT_EQ(u->phase, c.phase);
  EXPECT_DOUBLE_EQ(u->bias, -2.5);
  EXPECT_TRUE(u->switch_at_phase);
  EXPECT_FALSE(unpack_gait({4, 1, 2}));
}

TEST(Packing, StateRoundTrip)
{
  ActuatorStateData s{{1, -15}, {false, true}, 0.3, true, false};
  const auto u = unpack_state(pack(s));
  ASSERT_TRUE(u);
  EXPECT_EQ(u->voltage, s.voltage);
  EXPECT_EQ(u->clipped, s.clipped);
  EXPECT_TRUE(u->watchdog_tripped);
}
