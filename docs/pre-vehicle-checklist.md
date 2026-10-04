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
