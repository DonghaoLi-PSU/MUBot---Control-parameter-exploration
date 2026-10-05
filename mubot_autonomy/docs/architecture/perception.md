# Perception and steering

Sensor set: camera + range fan + IMU, all on the head. The head swing is
minimized by design, so each sensor is processed at its own rate. A moving
average over exactly one stroke period (1/f) removes the residual swing from
heading and yaw rate.

## One writer for the bias

Bias voltage b is the only steering input. Range and camera never set it.

```
range  ─► TTC → BACKWARD request; VFH+ → ψ_ref (avoid) ─┐
camera ─► ψ_ref (target) = ψ + θ; k(d); looming TTC ────┼─► arbiter ─► heading cascade ─► b
teleop ─► override ─────────────────────────────────────┘       ▲
IMU    ─► ψ, r (feedback) ──────────────────────────────────────┘
```

Arbiter priority (first active wins):

1. teleop override
2. BACKWARD (range TTC or camera looming TTC below T_crit, or range < d_min)
3. ψ_ref from avoidance (VFH+)
4. ψ_ref from the target
5. hold the last heading

## Per sensor

### IMU
- Estimation: complementary filter on yaw (gyro integration corrected by the
  absolute yaw), gyro-bias estimate while swimming straight, 1/f moving
  average on ψ and r.
- Control: cascade. Outer loop r_ref = K_ψ · wrap(ψ_ref − ψ), limited to
  ±r_max. Inner loop b = K_p·(r_ref − r) + K_i∫(r_ref − r), clipped to ±b_max
  with anti-windup.
- Tuning: apply bias steps first (`tools/bias_step_test.py`), fit first order
  + about one stroke of delay, then tune the PI.

### Range fan (5 beams, ±30°)
- Estimation: 3-sample median per beam, per-sector Kalman filter on distance d
  and closing speed ḋ, polar histogram.
- Decisions: TTC = d / (−ḋ) < T_crit or d < d_min → BACKWARD. VFH+ selects the
  free gap closest to the goal direction → ψ_ref. If everything ahead is
  blocked, turn in place at fixed bias to scan.

### Camera
- Detection: HSV blob in simulation, ArUco markers in a real tank (colour
  fades underwater). Bearing θ = atan((u − cₓ)/fₓ), distance d = fₓ·D / w,
  using `camera_info` for fₓ, cₓ.
- Tracking: constant-velocity Kalman filter on θ and d; lost after 0.5 s.
- Looming TTC = w / ẇ, a second BACKWARD trigger that covers what the 5 beams
  miss. Needs textured surfaces.
- Control: ψ_ref = ψ + θ to the heading cascade; amplitude scale k(d) slows the
  approach.

## State machine

```
FORWARD ──|ψ_ref − ψ| > 5°──► TURN ──|ψ_ref − ψ| < 2°──► FORWARD
   │                           │
   └──── TTC < T_crit ─────────┴────► BACKWARD ──clear or t > T_back──► ESCAPE TURN
                                                                          │
FORWARD ◄──── heading changed ≥ 60° and front clear ──────────────────────┘
```

Every transition takes effect at the next gait-phase zero crossing. Bias
changes are ramped over about one stroke.

## Start values

| Parameter | Value |
|---|---|
| Range max used / d_min | 1.0 m / 0.08 m |
| T_crit | 1.5 s |
| VFH+ bins / gap width | 5° / body width + 2 cm |
| Target HSV | red: H 0–10 or 170–180 |
| Target diameter D | 0.05 m |
| Target lost after | 0.5 s |
| Inner PI rate | 20 Hz |
| b_max | 15 V − max e_j |
