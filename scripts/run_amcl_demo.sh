#!/usr/bin/env bash
# Bring up the headless simulator and the installed, real Nav2 AMCL process.
set -eo pipefail
jev_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$jev_root/install/setup.bash" ]]; then
  source "$jev_root/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
jev_logs="${JEV_DEMO_LOG_DIR:-/tmp/jev-amcl-demo}"
mkdir -p "$jev_logs"
jev_pids=()
cleanup() {
  trap - EXIT INT TERM
  for jev_pid in "${jev_pids[@]}"; do kill -TERM "$jev_pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
python3 "$jev_root/scripts/demo_simulator.py" >"$jev_logs/simulator.log" 2>&1 &
jev_pids+=("$!")
"$(ros2 pkg prefix nav2_amcl)/lib/nav2_amcl/amcl" --ros-args --params-file "$jev_root/src/jev_localization_recovery/config/demo_amcl.yaml" >"$jev_logs/amcl.log" 2>&1 &
jev_pids+=("$!")
"$(ros2 pkg prefix nav2_lifecycle_manager)/lib/nav2_lifecycle_manager/lifecycle_manager" --ros-args -r __node:=demo_lifecycle_manager \
  -p autostart:=true -p 'node_names:=[amcl]' >"$jev_logs/lifecycle.log" 2>&1 &
jev_pids+=("$!")
printf 'Real AMCL demo running in ROS domain %s; logs: %s\n' "$ROS_DOMAIN_ID" "$jev_logs"
printf 'Start the monitor separately. Inject a wrong estimate with: python3 scripts/demo_simulator.py --initialpose 9.8 3.5 1.0\n'
wait -n "${jev_pids[@]}"
