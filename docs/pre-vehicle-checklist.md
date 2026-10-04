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

## Verification still owed

- [ ] **`is_autonomous_mode_available` has never become true.** The diagnostics point at
      localization rate checks (`topic_rate_check/transform`, `accuracy`,
      `sensor_fusion_status` all ERROR). Control commands are computed and pass the
      gate, but the system will not hand over. Re-check now that CPU is no longer
      saturated — this may already be resolved.
- [ ] **Re-measure the load with real sensor drivers.** The replay harness accounts for
      83.5% CPU today and disappears on the vehicle, but the nebula drivers will add
      back an unknown amount.
- [ ] **Reproduce and pin down the planning segfault.** `behavior_planning_container`
      died once (SIGSEGV) with the goal 14 m from the end of the lane while
      `goal_planner` was active. A mid-lane goal has been reliable since, but the
      comparison was never clean — in another run `planning_evaluator` segfaulted under
      the same settings.
- [ ] **Exercise the emergency-stop path on its own** before the first drive. On EZ10 a
      steering rate above 0.20 rad/s triggers a vehicle-level emergency stop.
