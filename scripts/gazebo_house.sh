#!/usr/bin/env bash
# Build/start the isolated environment, then launch the complete house demo.
set -eo pipefail
jev_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
jev_gui=true
jev_provider=mock
jev_key_file=''
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) jev_gui=false; shift ;;
    --jev) jev_provider=jev; shift ;;
    --key-file)
      [[ $# -ge 2 ]] || { echo '--key-file needs a path' >&2; exit 2; }
      jev_key_file="$2"; jev_provider=jev; shift 2 ;;
    *) echo 'Usage: scripts/gazebo_house.sh [--headless] [--jev] [--key-file PATH]' >&2; exit 2 ;;
  esac
done
jev_secret_args=()
if [[ "$jev_provider" == jev ]]; then
  if [[ -z "$jev_key_file" && -n "${TYPESAFE_API_KEY:-}" ]]; then
    # Pass the environment variable by name, never its contents in command args.
    jev_secret_args=(-e TYPESAFE_API_KEY)
  else
    jev_key_file="${jev_key_file:-${TYPESAFE_API_KEY_FILE:-$jev_root/API_KEY}}"
    jev_key_file="$(realpath -e -- "$jev_key_file")"
    case "$jev_key_file" in
      "$jev_root/"*) jev_secret_args=(-e "TYPESAFE_API_KEY_FILE=/ws/${jev_key_file#"$jev_root/"}") ;;
      *) echo 'Key file must be inside the mounted workspace, or export TYPESAFE_API_KEY.' >&2; exit 2 ;;
    esac
  fi
fi
if ! docker image inspect jev-recovery:gazebo >/dev/null 2>&1; then
  docker build -f "$jev_root/docker/gazebo-humble.Dockerfile" -t jev-recovery:gazebo "$jev_root"
fi
if ! docker container inspect jev-gazebo >/dev/null 2>&1; then
  jev_display_args=()
  if [[ -n "${DISPLAY:-}" && -f "${XAUTHORITY:-}" ]]; then
    jev_display_args=(--hostname "$(hostname)" -e "DISPLAY=$DISPLAY"
      -e XAUTHORITY=/tmp/jev.xauth -e LIBGL_ALWAYS_SOFTWARE=1
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "$XAUTHORITY:/tmp/jev.xauth:ro")
  elif [[ "$jev_gui" == true ]]; then
    echo 'Desktop display/Xauthority unavailable. Use --headless or run from your desktop terminal.' >&2
    exit 1
  fi
  docker run -d --name jev-gazebo --shm-size=512m -v "$jev_root:/ws" -w /ws \
    "${jev_display_args[@]}" jev-recovery:gazebo >/dev/null
fi
docker start jev-gazebo >/dev/null
docker exec jev-gazebo bash -lc \
  'source /opt/ros/humble/setup.bash && colcon --log-base log_gazebo build --build-base build_gazebo --install-base install_gazebo --symlink-install --packages-select jev_localization_recovery'
docker exec -it "${jev_secret_args[@]}" jev-gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && source /ws/install_gazebo/setup.bash && exec flock -n -E 73 /tmp/jev-house.lock ros2 launch jev_localization_recovery gazebo_house.launch.py gui:=$jev_gui rviz:=$jev_gui selector_provider:=$jev_provider"
