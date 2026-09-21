#!/usr/bin/env bash
# Launch a single isolated Jev mission environment and local dashboard.
set -eo pipefail
jev_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
jev_gui=true
jev_auto=false
jev_seed=42
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) jev_gui=false; shift;;
    --autostart) jev_auto=true; shift;;
    --seed) jev_seed="$2"; shift 2;;
    *) echo 'Usage: scripts/mission_demo.sh [--headless] [--autostart] [--seed INTEGER]' >&2; exit 2;;
  esac
done
[[ "$jev_seed" =~ ^[0-9]+$ ]] || { echo 'Seed must be a nonnegative integer'; exit 2; }
jev_secret_args=()
if [[ -n "${TYPESAFE_API_KEY:-}" ]]; then
  jev_secret_args=(-e TYPESAFE_API_KEY)
else
  jev_key_file="$(realpath -e -- "${TYPESAFE_API_KEY_FILE:-$jev_root/API_KEY}")"
  case "$jev_key_file" in
    "$jev_root/"*) jev_secret_args=(-e "TYPESAFE_API_KEY_FILE=/ws/${jev_key_file#"$jev_root/"}");;
    *) echo 'Key file must be in the workspace, or export TYPESAFE_API_KEY.' >&2; exit 2;;
  esac
fi
if ! docker image inspect jev-recovery:gazebo >/dev/null 2>&1; then
  docker build -f "$jev_root/docker/gazebo-humble.Dockerfile" -t jev-recovery:gazebo "$jev_root"
fi
if ! docker container inspect jev-mission >/dev/null 2>&1; then
  jev_display_args=()
  if [[ -n "${DISPLAY:-}" && -f "${XAUTHORITY:-}" ]]; then
    jev_display_args=(--hostname "$(hostname)" -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/jev.xauth
      -e LIBGL_ALWAYS_SOFTWARE=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$XAUTHORITY:/tmp/jev.xauth:ro")
  elif [[ "$jev_gui" == true ]]; then
    echo 'Desktop display unavailable; use --headless.' >&2; exit 1
  fi
  docker run -d --name jev-mission --shm-size=512m -e ROS_DOMAIN_ID=89 \
    -p 127.0.0.1:8765:8765 -v "$jev_root:/ws" -w /ws "${jev_display_args[@]}" jev-recovery:gazebo >/dev/null
fi
docker start jev-mission >/dev/null
docker exec jev-mission bash -lc \
 'source /opt/ros/humble/setup.bash && colcon --log-base log_mission build --build-base build_mission --install-base install_mission --symlink-install --packages-up-to jev_localization_recovery'
echo 'Dashboard: http://localhost:8765'
docker exec -i "${jev_secret_args[@]}" jev-mission bash -lc \
 "source /opt/ros/humble/setup.bash && source /ws/install_mission/setup.bash && exec flock -n -E 73 /tmp/jev-mission.lock ros2 launch jev_localization_recovery jev_mission.launch.py gui:=$jev_gui rviz:=$jev_gui seed:=$jev_seed autostart:=$jev_auto"
