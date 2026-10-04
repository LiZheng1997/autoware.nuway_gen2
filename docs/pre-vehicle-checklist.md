# Before the Orin goes on the vehicle

State as of 2026-10-04. Everything below is either measured on the bench or a known
gap; nothing here is assumed.

## What already works, replayed end to end

Full chain verified at **1.0x real time**, zero process deaths, with the self-generated
lanelet2 map:

```
route (latched)                       6 segments
behavior_planning/path                10.03 Hz
lane_driving/trajectory               10.06 Hz
/planning/trajectory                  10.06 Hz, 165 points
/control/command/control_cmd          67.05 Hz
trajectories dropped as late          0
CPU                                   6.5 of 12 cores
```

Localization initialises from GNSS on the first attempt, repeatedly (8 runs in a row).
CenterPoint, ground segmentation and the EKF all keep full frame rate — see
`docs/performance-on-orin.md`.

## Blocking — must be done before driving

### Hardware / data interfaces

- [ ] **GNSS antenna is open circuit.** NovAtel reports `antenna_is_open: true` and
      `gps_week_num: 0`. This is a physical fault; without a fix there is no GNSS pose
      and therefore no localization initialisation.
- [ ] **Real steering feedback.** `/vehicle/status/steering_status` is currently
      *synthesised* by `nuway/replay/bag_to_autoware.py` from yaw rate and speed
      (bicycle model). That exists only so the control chain could be verified offline.
      On the vehicle it must come from CAN `0x193`. Without it `trajectory_follower`
      sits in "Control is skipped since input data is not ready" and emits nothing.
- [ ] **SBG start-up path.** Still not established. Decide whether SBG or the NovAtel
      is the IMU/GNSS source and wire exactly one of them into the sensing launch.
- [ ] **Vehicle interface.** `canbridge_cpp@autoware` is already proven on the vehicle
      but has never been connected to this Autoware build; every run so far used
      `launch_vehicle_interface:=false`.

### Configuration to carry over

- [ ] `nuway/setup_env.sh` now exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
      `CYCLONEDDS_URI`. **Set `NUWAY_DDS_IFACE` to the NIC that reaches MainPC** if
      topics must leave the Orin; leave it at `lo` if everything runs on the Orin.
- [ ] `sudo cp nuway/60-autoware-dds.conf /etc/sysctl.d/ && sudo sysctl --system`
      (socket buffers; Cyclone's 10 MB receive buffer needs it).
- [ ] Run `patches/apply.sh` after every `vcs import` — it is idempotent and now also
      disables traffic-light recognition.

## Needed, not strictly blocking

### Map

- [ ] The vector map is **one 282 m single lane** generated from a recorded drive. It
      has no junctions, stop lines, crosswalks or neighbouring lanes. A real operating
      route has to be drawn properly.
- [ ] Point-cloud maps longer than roughly 300 m fail NDT acceptance because of
      non-rigid drift. Either segment the operating route or move to a SLAM pipeline
      with loop closure and pose-graph optimisation.
- [ ] When generating a lane from a recording, check first whether that stretch was
      driven forwards or backwards — `nuway/mapping/find_forward_window.py`. The lane
      must follow the vehicle heading, not the direction of travel. The campus bag is
      reverse for its first 460 s, which is exactly the part the current map was built
      from.

### Calibration

- [ ] `vehicle_info.param.yaml` is still byte-identical to `sample_vehicle`. Measure the
      real dimensions.
- [ ] Confirm the lidar and IMU extrinsics are measured values rather than inherited.
- [ ] Re-check that `max_steer_angle` (equivalent single-track) and the CAN physical
      limit of ±0.3141 rad stay consistent on the real vehicle.

## The kinematic model does not match the vehicle (found 2026-10-04)

Autoware's lateral controller runs `vehicle_model_type: "kinematics"`, whose core line is

```cpp
// autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
double delta_r = atan(m_wheelbase * m_curvature);     // delta = atan(L * kappa)
```

That is the ordinary front-steer bicycle model, referenced to the **rear axle**. EZ10 steers
both axles at equal and opposite angles, so its geometry is `R = L / (2*tan d)` and the
point that follows the path is the **vehicle centre**, not the rear axle.

The capability this buys is real: at the CAN limit of 0.3141 rad the minimum turning radius
is `2.79 / (2*tan 0.3141) = 4.26 m`, where a front-steer vehicle of the same wheelbase
would need 8.51 m. But for a *given* target curvature the physical angle each wheel needs is
**smaller**, not larger:

```
target curvature kappa
  Autoware (front-steer model)  d_aw   = atan(L*kappa)
  EZ10 needs (dual-axle)        d_phys = atan(L*kappa / 2)
  hence  tan(d_phys) = tan(d_aw) / 2,  d_phys < d_aw
```

### The conversion is missing in the bridge

`nuway_can/src/can_drive.cpp` takes Autoware's command straight through:

```
156  angular_ = msg->lateral.steering_tire_angle;   // equivalent single-track angle
237  selected_angular_ = -selected_angular_;
241  clamp to +/-0.28 rad                            // MAX_ANGULAR_VALUE
247  rate limit rad_per_frame_ = 0.004 per frame @ 50 Hz
258  angular_scaled = angular_target * 10000
267  front =  angular_scaled
268  rear  = -angular_scaled                         // opposite signs, correct
```

No division by two anywhere. The equivalent single-track angle is written into the physical
wheel setpoint, so the vehicle turns at roughly twice the intended curvature. The error is
not a constant scale, it changes sign across the clamp:

| commanded d_aw | what the vehicle does |
|---|---|
| below 0.28 rad | curvature about 2x the request - turns too tightly |
| above 0.28 rad | saturates at 0.28 -> R about 4.84 m, while Autoware assumes d=0.70 means R about 3.30 m - turns too wide |

### Command and feedback use different conventions

`nuway_can/src/can_status.cpp:188` reports

```cpp
steer.steering_tire_angle = (-front + rear) / 2.0;
```

which is the **physical** wheel angle, while Autoware expects `steering_status` in the same
convention as its command (equivalent single-track), i.e. about twice as large. The MPC will
read "commanded 0.2, measured 0.1" as steering lag and keep increasing the command - a
positive feedback that pushes the angle toward saturation.

### One coincidence worth knowing

`rad_per_frame_ = 0.004` at 50 Hz is exactly 0.2 rad/s, which is exactly the rate that
triggers a vehicle-level emergency stop. This is the limiter that actually binds (not
`vehicle_cmd_gate`, which permits 1.0 rad/s), but it leaves no margin at all.

### This needs reconciling before it is treated as a defect

`canbridge_cpp@autoware` is recorded as having run on the real vehicle, and the
planning-control chain of `autoware_on_nUWAy` as having driven. The code reads as above.
Either the error stayed small enough to go unnoticed at 1.3 m/s with gentle curvature, or
"ran on the vehicle" meant something short of closed-loop path tracking, or there is
compensation somewhere this analysis has not found. Ask before changing anything.

- [ ] **Bench test that settles it.** With the vehicle powered and the driven wheels off the
      ground or otherwise safe, command a known `steering_tire_angle` and read back both the
      raw CAN front/rear setpoints and `/vehicle/status/steering_status`. That measures the
      command convention and the feedback convention in one go, without moving the vehicle.
- [ ] **Constant-radius drive.** At low speed in a clear area, command a fixed curvature and
      measure the actual turning radius. If it comes out at half the requested radius, the
      missing factor of two is confirmed.

## Steering limits do not match the vehicle (found 2026-10-04)

`VEHICLE_LIMITS.md` records that a steering rate above **+0.20 rad/s triggers a full
vehicle emergency stop**. `vehicle_cmd_gate.param.yaml` currently permits five times
that, and a steering angle beyond what the vehicle declares:

| parameter | configured | vehicle limit |
|---|---|---|
| `steer_rate_lim_for_steer_cmd` | 1.0 rad/s | **0.20 rad/s** (emergency stop) |
| `steer_cmd_lim` | 1.0 rad | 0.70 rad (`vehicle_info.max_steer_angle`) |

Measured on the gate's own output (`/control/command/control_cmd`, 2001 commands over
60 s at 33.4 Hz):

- steering angle: median 0.0199 rad, p95 and max both exactly **0.7000** — the
  controller saturates at max_steer_angle
- rate implied by successive angles: p95 **0.3237 rad/s**, max **1.7323 rad/s**,
  **7.5% of commands above 0.20 rad/s**
- `steering_tire_rotation_rate` carried in the message is **always 0.0** — Autoware's
  controller never fills that field, so anything downstream that reads it to judge rate
  will read zeros. The gate rate-limits on the angle difference instead.

Caveat: this was measured in replay, where the ego is driven by the bag rather than by
the controller, so lateral error is large and the command saturates. Closed-loop values
will be far gentler. What it does establish is that the gate's 1.0 rad/s limit gives no
protection at the 0.20 rad/s threshold, and that commands above it do get through.

- [ ] Lower `steer_rate_lim_for_steer_cmd` below 0.20 rad/s with margin, and
      `steer_cmd_lim` to at most 0.70, then re-measure closed loop on the vehicle.

## Verification still owed

- [x] **Autonomous-mode diagnostics — resolved.** Under CycloneDDS the localization
      ERRORs (`topic_rate_check/transform`, `accuracy`) and the control ERROR
      (`topic_rate_check/trajectory_follower`) are all gone; `sensor_fusion_status` is
      down to a WARN and `/autoware/modes/autonomous` from ERROR to WARN. They were
      symptoms of CPU starvation, not of sensors or algorithms.
- [ ] **Engagement itself is still untested.** The remaining refusal is correct
      behaviour, not a fault: `operation_mode_transition_manager` reports *"Engage
      unavailable: enable_engage_on_driving is false, and the vehicle is not
      stationary"*, and stationary means below 1 cm/s (`state.cpp:229`). The replay is a
      recording of a drive, so that condition never holds. Overriding the parameter at
      runtime does **not** work — `state.cpp:41` reads it once with
      `declare_parameter` into a member. Test it on the vehicle from a standstill, or
      with the parameter file edited for a bench run.
- [ ] **Re-measure the load with real sensor drivers.** The replay harness accounts for
      83.5% CPU today and disappears on the vehicle, but the nebula drivers will add
      back an unknown amount.
- [x] **Planning segfault — goal position ruled out.** Re-run with the goal at the exact
      position that crashed before (map 15.1, -0.1, i.e. 14.1 m from the end of the
      lane): no deaths, route set 5 times, chain healthy at trajectory 10.14 Hz and
      control_cmd 67.58 Hz. `goal_planner` went *further* than in the crashing run —
      it generated 9 pull-over candidates, where the crashing run printed one line and
      died. The crash correlates with the saturated rmw_fastrtps_cpp environment, not
      with where the goal sits. Not proof of absence, but the attribution is refuted.
- [ ] **Exercise the emergency-stop path on its own** before the first drive.
