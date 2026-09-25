# Bringing the Orin up on the vehicle

The order below exists because each step's failure mode is invisible once the
next one is running. Work through it in sequence and stop at the first red mark.

`nuway/vehicle_preflight.sh` automates the checks. It is read-only unless you
pass `--apply-net`.

---

## 1. Know which vehicle you are on

The two vehicles do not carry the same sensors, and the correction path differs
completely between them.

| | Vehicle with SBG | Vehicle with NovAtel |
|---|---|---|
| GNSS/INS | SBG (serial, 921600 on `/dev/ttyUSB0`) | NovAtel **FlexPak6** (OEM628, OEM6 generation) |
| IMU | part of the SBG | **XSENS MTi-10** - a plain IMU, no attitude solution |
| ROS driver | `sbg_driver` | **`ros-humble-novatel-gps-driver`** (swri, tested on OEM628). *Not* `novatel_oem7_driver`, which is OEM7 only. XSENS has no apt package and must be built from source. |
| RTK corrections | ROS side: the `ntrip` package publishes `/rtcm`, and `sbg_params.yaml` already sets `rtcm.subscribe: true` and `nmea.publish: true` | **Receiver side**: the FlexPak6 has Ethernet and its own NTRIP v1/v2 client, so put it on the vehicle switch and let it fetch corrections itself. ROS stays out of the correction path entirely. |
| Heading | SBG supplies INS orientation | Single antenna, so no INS heading. Keep `use_gnss_ins_orientation` false and let `gnss_poser` derive heading from successive positions. |

## 2. Network

One flat `192.168.5.0/24` carries both the sensors and the internet, because the
4G router sits on the same subnet. The Orin therefore needs only its one
Ethernet port.

```
Orin        192.168.5.29     <- hard-coded as host_ip in lidar.launch.xml
front lidar 192.168.5.28     data port 2369
rear lidar  192.168.5.27     data port 2368
MainPC      <to be confirmed>  runs canbridge, owns the CAN interface
4G router   <to be confirmed>  also the default gateway
```

```bash
ROUTER_IP=<router> bash nuway/vehicle_preflight.sh --apply-net
```

It asks for confirmation first, because changing the address drops every SSH
session. Run it from a local terminal. The change does not survive a reboot;
create an `nmcli` connection to make it permanent.

## 3. Run the lidar drivers on the Orin, not over ROS from MainPC

Taking decoded point clouds from MainPC costs about 44 Mbit/s, where reading the
sensors' raw UDP directly costs about 6.7 Mbit/s - a factor of 6.5, measured from
the campus bag at 12,500 points per frame and 10 Hz on two sensors.

Bandwidth is the smaller reason. The decisive one is that `common_cuda_sensor_launch`
exists to compose the CUDA preprocessor into the same container as the nebula
driver, so multi-megabyte clouds stay in-process. A cloud arriving over the
network cannot be zero-copy, and it arrives as a generic `PointCloud2` rather
than the `PointXYZIRCAEDT` that chain expects - which would drag the offline
replay adapter onto the vehicle for no reason. Timestamps are the third reason:
decoding locally keeps the point cloud clock independent of MainPC's.

Bring the lidars up on their own first and confirm both publish before starting
anything else.

## 4. ROS_DOMAIN_ID

The vehicle uses **domain 5**, and the Orin and MainPC must agree or they simply
never see each other - there is no error, the topics just never appear.

**Replay tests must use a different domain.** Replaying a bag on domain 5 while
MainPC is live puts recorded `/vehicle/status/velocity_status` alongside the real
one, the EKF consumes whichever arrives, and the resulting behaviour is very hard
to reason about. Use domain 71, which the mapping scripts already default to.

## 5. Clock synchronisation

Both machines must follow the same time source. If they drift apart, the
timestamps on `/vehicle/status/*` will not line up with the sensor stream, and
the failure looks exactly like the 18.2 s offset in the campus bag did: NDT
rejects every frame with `Validation error` while the EKF keeps publishing at
40 Hz and the stack looks healthy.

Without reliable internet on the vehicle, have one machine serve NTP to the
other. Agreeing with each other matters more than being right in absolute terms.

## 6. Order of bring-up

1. `bash nuway/vehicle_preflight.sh` - sections 1 to 3 must be green before anything else
2. Lidars only. Confirm both topics publish at 10 Hz and the CUDA container is alive.
3. Preflight sections 4 and 5: MainPC topics visible, clock offset small.
4. Localization against a map, replayed from a bag, **on domain 71**.
5. Only then switch to domain 5 and run against live vehicle data.

## 7. What to check at each stage

| Stage | Signal | Failure looks like |
|---|---|---|
| Lidar | both `/sensing/lidar/*/pointcloud_raw_ex` at 10 Hz | silence, usually the wrong `host_ip` |
| Localization | NVTL median above 2.3, NDT pose near 10 Hz | NVTL samples present but zero pose frames - the map, not the tooling |
| Vehicle interface | `/vehicle/status/velocity_status` arriving | EKF publishes but dead-reckons; NDT reports `Validation error` |
| RTK | `/gps/fix` status 0 → 2, covariance to centimetres | stuck at 1 usually means the antenna, not NTRIP, since the correction stream can be verified independently |

`nuway/replay/ndt_verify.sh` and `nuway/replay/collect_ndt.py` produce the
localization numbers; see `docs/mapping-and-localization.md` for what they mean
and which thresholds Autoware itself gates on.
