#!/usr/bin/env bash
set -eu
if ! command -v ros2 >/dev/null; then
  echo 'ros2 is unavailable. Source a complete ROS 2 installation first.' >&2
  exit 1
fi
echo "ROS distro: ${ROS_DISTRO:-unknown}"
python3 --version
echo 'Available navigation packages:'
if command -v rg >/dev/null; then
  ros2 pkg list | rg '^(nav2_|navigation2|turtlebot)' || true
else
  ros2 pkg list | grep -E '^(nav2_|navigation2|turtlebot)' || true
fi
echo 'Topics and message types:'
ros2 topic list -t
echo 'Services and service types:'
ros2 service list -t
echo 'Nodes:'
ros2 node list
echo 'Simple commander:'
ros2 pkg prefix nav2_simple_commander || true
