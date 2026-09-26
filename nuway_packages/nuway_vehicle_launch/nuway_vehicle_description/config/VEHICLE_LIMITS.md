# EZ10 Gen2 CAN-side limits (reference for configuration)

Source: `canbridge_cpp/GEN2-Docs/EM-EZ10PROD-IDD-00011-EN-rev21.06-*.pdf`

| Item | CAN Signal | Frame ID | Range | Notes |
|---|---|---|---|---|
| Front wheel steering angle | `c_PCN_Front_Steering_Setpoint` | 0x193 byte 4-5 | −0.3141 ~ +0.3141 rad (±18°) | Factor 0.0001, i.e. rad×10000 |
| Rear wheel steering angle | `c_PCN_Rear_Steering_Setpoint` | 0x193 byte 6-7 | −0.3141 ~ +0.3141 rad | Front and rear **must have opposite signs** — same sign is prohibited (prevents lateral translation) |
| Speed | Traction command | — | −7.0 ~ +7.0 m/s | Factor 0.001, +1000 ⇔ 1 m/s; negative values mean reverse |
| Acceleration | — | — | 0.3 ~ 1.0 m/s² | Out-of-range values fall back to the default of 0.5 |
| Deceleration | — | — | −1.5 ~ −0.3 m/s² | Out-of-range values fall back to the default of −0.8 |
| Steering rate | — | — | Varies with speed (see the curve in the doc) | **Exceeding +0.20 rad/s triggers a full vehicle emergency stop** |

## How these values relate to `vehicle_info.param.yaml`

**The ±0.3141 rad in the table above is the physical wheel steering angle; the
`max_steer_angle` in `vehicle_info.param.yaml` is an equivalent single-track (bicycle) model parameter. These are not the same quantity and should not be compared directly.**

When EZ10 steers both axles at equal and opposite angles, the turning radius is `R = L / (2·tanδ)`, whereas single-axle steering gives
`R = L / tanδ`. For the same radius, the equivalent single-track steering angle is `δ_eq = atan(2·tanδ)`:

```
δ_phys = 0.3141 rad (18°)  →  δ_eq = atan(2·tan 0.3141) ≈ 0.576 rad (33°)
```

The current `max_steer_angle: 0.70` configuration is consistent with this order of magnitude and **has been confirmed by real-vehicle testing**,
so it can be kept as-is. This document only records the raw CAN-side limits, for reference when configuring
longitudinal control clamping, steering-rate margins, and similar parameters.

## Practical impact on Autoware configuration

1. **Longitudinal control's acceleration/deceleration clamps** should stay within the ranges in the table above; otherwise the vehicle side will rewrite the command to the default value
   (0.5 / −0.8 m/s²), and actual behavior won't match what was planned.
2. **Steering rate** must keep a margin — exceeding +0.20 rad/s triggers a full vehicle emergency stop.
3. Once the steering angle exceeds 3.94°, the vehicle will **automatically limit its speed** to stay within the maximum lateral acceleration; if the planned speed profile doesn't account for this, tracking will persistently lag.
4. EZ10 steers on **both the front and rear axles** (the CAN signals include `s_MECU_Front_Steering_Axle_Select`,
   `FSC`, `axle_2`), while Autoware's default is a front-wheel-steering model — any parameter involving turning radius
   needs to be converted using the equivalence above; don't plug in the physical wheel angle directly.

## Geometry

`wheel_base` / `wheel_tread` / front and rear overhang / vehicle height, etc. match Autoware's `sample_vehicle`
(only `max_steer_angle` differs). The two CAN documents are command manuals and don't include geometry,
so they can't be used to verify this. To confirm, cross-check against `mirror.param.yaml` (lateral ±1.4 m, measured on the real vehicle)
or measure the actual vehicle.

## Steering mode: dual-axle, and deliberately not switchable

The vehicle supports three steering modes. `nuway_can/include/nuway_can/can_drive.hpp`
declares them:

```cpp
enum GearMode { FRONT = 1, REAR = 2, DUAL = 0 };
```

but the selection is commented out in `can_drive.cpp`, under the note
`// Commenting Down GearMode selection for autoware`, and the angles are written
unconditionally with opposite signs:

```cpp
convert_16_to_8(&front_msb, &front_lsb,  angular_scaled);
convert_16_to_8(&rear_msb,  &rear_lsb,  -angular_scaled);
```

So under Autoware the vehicle always steers both axles in counter-phase. **Keep it
that way.** The reasoning, so that nobody has to reconstruct it:

**`max_steer_angle` is only valid for one mode.** The equivalence
`δ_eq = atan(2·tanδ)` above assumes both axles steer. It does not hold otherwise:

| Mode | Minimum radius | Equivalent single-track angle | Is `max_steer_angle: 0.70` right? |
|---|---|---|---|
| DUAL (current) | `L/(2·tanδ)` = 4.29 m | ≈ 0.576 rad | yes, same order, and vehicle-validated |
| FRONT or REAR | `L/tanδ` = 8.59 m | 0.3141 rad | **no - overstates the vehicle by 2.2×** |

**Autoware cannot change the vehicle model while running.**
`vehicle_info.param.yaml` is resolved through
`find-pkg-share $(var vehicle_model)_description` by the control, planning,
perception and sensing components, each at launch. There is no runtime path to a
different `wheel_base` or `max_steer_angle`. Supporting a mode switch therefore
means either restarting the stack against a second vehicle model, or leaving the
planner working from figures the vehicle cannot deliver.

**The second option fails silently**, which is why it is worth stating as a rule
rather than leaving to judgement. Nothing raises an error: the planner simply
asks for curvature the vehicle cannot produce, and it shows up as persistent
tracking lag, cutting corners, and in the worst case leaving the path.

Dual is also the mode you want here on its own merits - half the turning radius,
4.29 m against 8.59 m, which is what makes a turnaround possible in roughly 8.6 m
of width instead of 17.2 m.

If a single-axle mode is ever genuinely needed, for a docking manoeuvre say,
handle it outside Autoware's control path rather than as a runtime switch inside
the autonomous stack. And if the `GearMode` selection is ever restored, treat
`max_steer_angle` as part of that change, not as a separate question.

Note that the commented-out block still carries
`// TODO: Double check the direction of the steering.` The current behaviour is
what we want, but it arrived by someone disabling code they were unsure of rather
than by decision - which is the reason for writing this down.
