# Environment inspection: 2026-09-20

Initial workspace: empty apart from an existing zero-byte file named `,` (preserved).

Host: Ubuntu 22.04.5. `/opt/ros/foxy` and `/opt/ros/noetic` exist.
Foxy is incomplete: no `ros2` executable, Nav2, AMCL, or `nav2_simple_commander`.
Its `rclpy` extensions target Python 3.8; system Python is 3.10.12 and the default
Anaconda Python is 3.11.5. `colcon` exists. No running ROS/Gazebo nodes were found.
There was therefore no live AMCL service, scan, map, odometry, or velocity topic
to identify on the host. No source or host package was modified.

For verification, an isolated Docker container `jev-recovery-test` was created
from `ros:humble-ros-base-jammy`, with the workspace mounted at `/ws` and
`ROS_DOMAIN_ID=87`. It includes AMCL, map server, lifecycle manager, TF2, pytest,
and colcon. `docker/test-humble.Dockerfile` reproduces the runtime.
The container uses Docker's default network, not the host's robot network.

The production node discovers names by message type and recognizable basename
and requires a unique candidate. Explicit YAML names resolve ambiguous graphs.
It reports each binding. The relocalization service must be discovered as
`std_srvs/srv/Empty`; an undocumented service name is never assumed available.

ROS references used for compatibility checks:

- [ROS platform and Docker guidance](https://www.ros.org/blog/getting-started/)
- [Humble AMCL source and service creation](https://api.nav2.org/nav2-humble/html/amcl__node_8cpp_source.html)
- [Foxy AMCL source](https://raw.githubusercontent.com/ros-navigation/navigation2/foxy-devel/nav2_amcl/src/amcl_node.cpp)

## Gazebo extension

Container `jev-gazebo`, image `jev-recovery:gazebo`, domain 88, Ubuntu Jammy / ROS
Humble. Gazebo Classic 11.10.2, `turtlebot3_gazebo`, `turtlebot3_navigation2`, and
RViz2 are installed inside it. Official TurtleBot3 Burger model; custom generated
Jev house with matching occupancy grid. Both desktop windows were launched through
the host's X11 display, with cookie authentication mounted read-only.

Observed graph: `/scan` LaserScan, `/map` OccupancyGrid, `/amcl_pose`
PoseWithCovarianceStamped, `/odom` Odometry, `/cmd_vel` Twist, and `/clock`.
Odometry child frame is `base_footprint`; laser frame is `base_scan`. Recovery sends
Twist to `/recovery/cmd_vel`; demo support relays commands with a watchdog to `/cmd_vel`.
Observed AMCL Empty services: `/reinitialize_global_localization` and
`/request_nomotion_update`. Nav2 localization is launched; planner/controller goal
navigation is not. No Nav2 or AMCL sources were modified.

References: [ROBOTIS simulation guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/),
[official Humble simulation source](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble).


## Mission extension

`jev-mission` uses the same Gazebo image, a separate ROS domain **89**, and isolated
`build_mission`, `install_mission`, and `log_mission` directories. It adds real
Nav2 planning/control, recovery-free navigation behavior trees, the Jev mission
coordinator, and a dashboard published on host loopback port **8765**. The old
`jev-gazebo` and `jev-recovery-test` containers were stopped. The unrelated
`open-webui` container was left running. No host ROS installation was changed.
