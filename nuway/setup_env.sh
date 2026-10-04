_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$_HERE/install/setup.bash" ] && source "$_HERE/install/setup.bash"

export LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib:${LD_LIBRARY_PATH}
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:${PATH}

# ---- DDS ---------------------------------------------------------------------
# Autoware is tuned for CycloneDDS; rmw_cyclonedds_cpp ships with the ROS install but
# RMW_IMPLEMENTATION was never set, so Humble silently fell back to rmw_fastrtps_cpp.
# With 13 network interfaces on this box and ROS_LOCALHOST_ONLY=0, discovery across all
# of them cost 3.4 of 12 cores and kept the planning chain from ever meeting
# scenario_selector's 1.0 s delay gate. Measured at 1.0x replay on 2026-10-04:
#   fastrtps / all interfaces  -> 9.9 cores, planning produced nothing
#   cyclonedds / lo only       -> 6.5 cores, trajectory 10.06 Hz, control_cmd 67.05 Hz
# See nuway/cyclonedds.xml and docs/performance-on-orin.md.
#
# NUWAY_DDS_IFACE: keep "lo" while every node runs on the Orin. Set it to the NIC that
# reaches MainPC (e.g. eth0) if topics have to leave the box - name one interface either
# way, never let discovery fan out across all of them.
export NUWAY_DDS_IFACE="${NUWAY_DDS_IFACE:-lo}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$_HERE/nuway/cyclonedds.xml}"
