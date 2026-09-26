#!/bin/bash
# Record one data-collection run on the vehicle.
#
#   bash nuway/record_session.sh teach   loop_a    # VTR teach, also the FAST-LIO mapping pass
#   bash nuway/record_session.sh repeat1 loop_a
#   bash nuway/record_session.sh repeat2 loop_a
#
# It refuses to start unless every required topic is actually publishing, writes
# to the internal NVMe rather than the external disk, and records a metadata file
# next to the bag so the run can be interpreted months later.
#
# Environment:
#   CAMERAS=1        also record the two cameras (about 42 MB/s instead of 5.5)
#   OUT_ROOT=<dir>   where bags go (default /home/lz/vehicle_data)
#   ROS_DOMAIN_ID    must match whichever machine runs the sensor drivers
set -u
LABEL=${1:?usage: record_session.sh <teach|repeat1|repeat2|...> <route-name>}
ROUTE=${2:?usage: record_session.sh <label> <route-name>}
OUT_ROOT=${OUT_ROOT:-/home/lz/vehicle_data}
CAMERAS=${CAMERAS:-0}

# The five topics every downstream tool needs. FAST-LIO mapping, VTR teach/repeat
# and the Autoware localization replay all read exactly this set.
REQUIRED=(
  /lidar/velodyne/front/cloud
  /lidar/velodyne/rear/cloud
  /imu/data
  /gps/fix
  /can_twist_fb
)
OPTIONAL=(/CameraFront /CameraRear /tf_static)

ok(){   printf "  \033[32m✓\033[0m %s\n" "$*"; }
bad(){  printf "  \033[31m✗\033[0m %s\n" "$*"; }
hdr(){  printf "\n\033[1m%s\033[0m\n" "$*"; }

STAMP=$(date +%Y%m%d_%H%M%S)
DIR="$OUT_ROOT/${ROUTE}_${LABEL}_${STAMP}"

hdr "Pre-flight"
echo "  domain=${ROS_DOMAIN_ID:-0}  label=$LABEL  route=$ROUTE"

# Writing to a slow disk silently drops messages. Measured on this machine:
# internal NVMe 839 MB/s, external NTFS 37.5 MB/s, against 5.5 MB/s without
# cameras and about 42 MB/s with them - which the external disk cannot sustain.
FS=$(df -h --output=target "$(dirname "$OUT_ROOT")" 2>/dev/null | tail -1)
case "$OUT_ROOT" in
  /media/*|/mnt/*) bad "$OUT_ROOT looks like an external mount. Record to the NVMe and copy afterwards." ;;
  *) ok "writing to $OUT_ROOT" ;;
esac
AVAIL=$(df -BG --output=avail "$(dirname "$OUT_ROOT")" 2>/dev/null | tail -1 | tr -dc 0-9)
NEED=$([ "$CAMERAS" = 1 ] && echo 45 || echo 8)
[ "${AVAIL:-0}" -ge "$NEED" ] && ok "${AVAIL}G free, need about ${NEED}G for a 15 min run" \
                              || { bad "only ${AVAIL}G free, need about ${NEED}G"; exit 1; }

hdr "Topics"
LIVE=$(timeout 15 ros2 topic list 2>/dev/null)
MISSING=0
for t in "${REQUIRED[@]}"; do
  if echo "$LIVE" | grep -qx "$t"; then
    hz=$(timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 -oE "average rate: [0-9.]+" | cut -d" " -f3)
    [ -n "$hz" ] && ok "$t  ${hz} Hz" || { bad "$t advertised but silent"; MISSING=$((MISSING+1)); }
  else
    bad "$t not present"; MISSING=$((MISSING+1))
  fi
done
TOPICS=("${REQUIRED[@]}")
if [ "$CAMERAS" = 1 ]; then
  for t in "${OPTIONAL[@]}"; do
    echo "$LIVE" | grep -qx "$t" && { ok "$t"; TOPICS+=("$t"); } || printf "  \033[33m!\033[0m %s not present, skipping\n" "$t"
  done
fi
[ "$MISSING" -gt 0 ] && { bad "$MISSING required topics missing - start the sensor drivers first"; exit 1; }

hdr "Before you start driving"
cat <<'NOTE'
  - Stand still for at least 15 s after recording starts. FAST-LIO estimates
    gravity and the IMU biases from that window, and VTR inherits the benefit.
  - Park facing the same way for every run of a route. A VTR teach graph carries
    the heading the vehicle was parked with, so a repeat parked the other way is
    180 degrees out and will not localize.
  - On a repeat run, drive 0.5 to 1 m off the taught line on purpose. A repeat
    that retraces the teach exactly cannot demonstrate that VTR corrects anything.
  - Stand still for 10 s at the end, back at the start point.
NOTE
printf "\n  Press Enter to start recording, Ctrl-C to abort. "
read -r _

mkdir -p "$DIR"
cat > "$DIR/session.txt" <<META
label:        $LABEL
route:        $ROUTE
started:      $(date -Is)
domain:       ${ROS_DOMAIN_ID:-0}
cameras:      $CAMERAS
host:         $(hostname)
topics:       ${TOPICS[*]}
parking:      FILL IN - which way the vehicle was facing at the start
notes:        FILL IN - weather, traffic, anything unusual
META

hdr "Recording to $DIR"
echo "  Ctrl-C once to stop cleanly."
ros2 bag record -s mcap -o "$DIR/bag" "${TOPICS[@]}"

hdr "Done"
du -sh "$DIR" 2>/dev/null | sed "s/^/  /"
echo "  ⚠ Fill in the 'parking' and 'notes' lines in $DIR/session.txt while you still remember."
echo "  Copy to the external disk afterwards; do not record straight onto it."
