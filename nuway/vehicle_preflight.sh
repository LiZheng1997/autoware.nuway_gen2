#!/bin/bash
# Readiness check for bringing the Orin up on the vehicle.
# Read-only by default: it changes nothing unless --apply-net is passed.
#
#   bash nuway/vehicle_preflight.sh              check only
#   bash nuway/vehicle_preflight.sh --apply-net  also move eth0 onto the vehicle network
#
# ⚠ --apply-net drops every SSH session the moment the address changes, because
#   the address is the one you are connected over. Run it from a local terminal.
#
# Fill these in on the vehicle, or pass them in the environment:
ROUTER_IP=${ROUTER_IP:-192.168.5.1}        # 4G router, also the default gateway - TO BE CONFIRMED
MAINPC_IP=${MAINPC_IP:-192.168.5.10}       # MainPC, which runs canbridge - TO BE CONFIRMED
ORIN_IP=${ORIN_IP:-192.168.5.29}           # hard-coded as host_ip in lidar.launch.xml
IFACE=${IFACE:-eth0}
LIDAR_FRONT=${LIDAR_FRONT:-192.168.5.28}
LIDAR_REAR=${LIDAR_REAR:-192.168.5.27}
VEHICLE_DOMAIN=${VEHICLE_DOMAIN:-5}

ok(){ printf "  \033[32m✓\033[0m %s\n" "$*"; }
bad(){ printf "  \033[31m✗\033[0m %s\n" "$*"; FAIL=$((FAIL+1)); }
warn(){ printf "  \033[33m!\033[0m %s\n" "$*"; }
hdr(){ printf "\n\033[1m%s\033[0m\n" "$*"; }
FAIL=0

if [ "$1" = "--apply-net" ]; then
  echo "About to set $IFACE to $ORIN_IP/24 with gateway $ROUTER_IP."
  echo "This drops every SSH session immediately. Running from a local terminal? [yes/N]"
  read -r a; [ "$a" = "yes" ] || exit 1
  sudo ip addr flush dev "$IFACE"
  sudo ip addr add "$ORIN_IP/24" dev "$IFACE"
  sudo ip link set "$IFACE" up
  sudo ip route replace default via "$ROUTER_IP" dev "$IFACE"
  echo "Applied. This does not survive a reboot - create an nmcli connection to make it permanent."
fi

hdr "1. Network"
ip -br addr show "$IFACE" | sed "s/^/  /"
ip -br addr show "$IFACE" | grep -q "$ORIN_IP" \
  && ok "$IFACE is $ORIN_IP, the host_ip lidar.launch.xml expects" \
  || bad "$IFACE is not $ORIN_IP - the lidar drivers will receive nothing"
ip route show default | sed "s/^/  gateway: /"

hdr "2. Lidar reachability - decides whether the drivers can run on the Orin"
for ip in "$LIDAR_FRONT:front" "$LIDAR_REAR:rear"; do
  a=${ip%:*}; n=${ip#*:}
  ping -c2 -W2 "$a" >/dev/null 2>&1 && ok "$n lidar $a reachable" || bad "$n lidar $a unreachable"
done

hdr "3. Internet and NTRIP"
ping -c2 -W2 "$ROUTER_IP" >/dev/null 2>&1 && ok "router $ROUTER_IP reachable" || bad "router $ROUTER_IP unreachable"
getent hosts ntrip.data.gnss.ga.gov.au >/dev/null 2>&1 \
  && ok "AUSCORS resolves" || bad "DNS lookup failed - check the router's DNS"
timeout 8 bash -c "exec 3<>/dev/tcp/ntrip.data.gnss.ga.gov.au/2101" 2>/dev/null \
  && ok "AUSCORS caster reachable on 2101" || bad "cannot reach AUSCORS - no route to the internet"

hdr "4. ROS 2 with MainPC (domain $VEHICLE_DOMAIN)"
ping -c2 -W2 "$MAINPC_IP" >/dev/null 2>&1 && ok "MainPC $MAINPC_IP reachable" || bad "MainPC $MAINPC_IP unreachable"
if command -v ros2 >/dev/null 2>&1; then
  T=$(ROS_DOMAIN_ID=$VEHICLE_DOMAIN timeout 20 ros2 topic list 2>/dev/null)
  n=$(echo "$T" | grep -c "^/")
  echo "$T" | grep -qE "^/vehicle/status/velocity_status$" \
    && ok "/vehicle/status/velocity_status present - canbridge is publishing and the switch passes multicast" \
    || bad "/vehicle/status/velocity_status missing - without it gyro_odometer has no twist and the EKF free-runs"
  echo "  $n topics discovered on domain $VEHICLE_DOMAIN"
  for t in /vehicle/status/steering_status /vehicle/status/control_mode /vehicle/status/gear_status; do
    echo "$T" | grep -qx "$t" && ok "$t" || warn "$t not seen"
  done
else
  warn "ROS environment not sourced, skipping topic checks"
fi

hdr "5. Clock synchronisation - where a two-machine setup usually goes wrong"
timedatectl 2>/dev/null | grep -E "synchronized|NTP service" | sed "s/^/  /"
if command -v chronyc >/dev/null 2>&1; then
  chronyc tracking 2>/dev/null | grep -E "Reference ID|System time|Last offset" | sed "s/^/  /"
fi
if command -v ntpdate >/dev/null 2>&1; then
  o=$(ntpdate -q "$MAINPC_IP" 2>/dev/null | tail -1)
  [ -n "$o" ] && echo "  against MainPC: $o" || warn "cannot measure offset against MainPC - is it serving NTP?"
else
  warn "ntpdate not installed; needed to measure the offset against MainPC (sudo apt install ntpdate)"
fi
echo "  ⚠ If the two clocks disagree, /vehicle/status timestamps will not line up with the"
echo "    sensor stream. It looks exactly like the 18.2 s offset did: NDT reports Validation"
echo "    error on every frame while the EKF keeps publishing and the stack appears healthy."

hdr "6. Host configuration"
for k in net.core.rmem_max net.ipv4.ipfrag_time; do
  v=$(sysctl -n $k 2>/dev/null)
  case $k in
    net.core.rmem_max) [ "$v" = "2147483647" ] && ok "$k=$v" || bad "$k=$v, expected 2147483647 - install nuway/60-autoware-dds.conf";;
    *) [ "$v" = "3" ] && ok "$k=$v" || bad "$k=$v, expected 3";;
  esac
done
a=$(df -BG --output=avail / | tail -1 | tr -dc 0-9)
[ "${a:-0}" -ge 100 ] && ok "${a}G free" || bad "only ${a}G free, clear space before recording"

hdr "7. Sensor devices"
ls /dev/ttyUSB* 2>/dev/null | sed "s/^/  /" || warn "no USB serial devices - GNSS and IMU not connected or not powered"
ip -br link show can0 2>/dev/null | sed "s/^/  /"
echo "  CAN runs on MainPC, so the Orin does not need can0."

hdr "Result"
[ "$FAIL" -eq 0 ] && echo "  all checks passed, Autoware can be started" || echo "  $FAIL checks failed, fix them first"
exit $((FAIL > 0))
