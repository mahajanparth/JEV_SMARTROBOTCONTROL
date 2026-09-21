# JEV_SMARTROBOTCONTROL

A ROS 2 robotics demo in which a TurtleBot3 Burger starts at a random location in
a simulated house, localizes with AMCL, and navigates to a random destination.
Jev chooses mission and recovery actions and monitors navigation while Nav2 drives.
A browser dashboard shows the map, live Jev requests and responses, action
probabilities, motion ownership, and recovery results.

Start here for the current mission demo. Detailed behavior is in the
[mission guide](docs/mission-demo.md); earlier demos are documented separately in
[recovery-only instructions](docs/recovery-only.md).

## Requirements

- A Linux host with Docker installed, running, and accessible to your user.
  The development environment is Ubuntu 22.04; see the
  [environment notes](docs/environment.md) for the inspected setup.
- This repository checked out locally, with Bash available.
- Internet access for the first Docker build and live TypeSafe Jev API calls.
- A Jev API key.
- For Gazebo and RViz windows, an X11 display with `DISPLAY` set and a readable
  `XAUTHORITY` file. Use `--headless` if desktop rendering is unavailable.
- Host port `8765` available for the dashboard.

ROS 2 Humble, Gazebo Classic, TurtleBot3, AMCL, Nav2, and RViz run inside Docker.
You do not need to install or source ROS on the host. The launcher builds the
image on first use and builds the ROS packages on each launch.

## First run

Clone the repository and open it in a terminal:

```bash
git clone https://github.com/mahajanparth/JEV_SMARTROBOTCONTROL.git
cd JEV_SMARTROBOTCONTROL
docker info
```

Save the raw Jev key in a file named `API_KEY` at the repository root, without
quotes or a variable assignment. This file is excluded from Git and the Docker
build context. Keep credentials out of logs and commits.

Alternatively, export `TYPESAFE_API_KEY` in your shell, or set
`TYPESAFE_API_KEY_FILE` to a key file inside the repository. The environment
variable takes precedence over the file.

```bash
# Start Gazebo, RViz, and the dashboard. Keep this terminal open.
./scripts/mission_demo.sh
```

Open [the dashboard](http://localhost:8765), choose a seed, and click
**Start random mission**. The demo starts idle by default. To open the dashboard
from another terminal:

```bash
xdg-open http://localhost:8765
```

Gazebo shows the physical robot and house. RViz shows the estimated pose, laser
scan, map, and route. The dashboard shows mission progress and Jev activity.

Choose one of these alternatives when no mission launch is running:

```bash
# Start a mission immediately.
./scripts/mission_demo.sh --autostart --seed 42

# Simulation and dashboard without Gazebo/RViz windows.
./scripts/mission_demo.sh --headless

# Headless with immediate mission start.
./scripts/mission_demo.sh --headless --autostart --seed 7
```

The seed reproduces start and goal sampling. AMCL convergence and Jev choices can
still vary. The house is a custom 12 by 10 metre map with rooms and furniture.

## Stop and restart

The dashboard's **Stop mission** button stops robot motion and keeps the simulation
and dashboard available. To shut down the entire demo, use another terminal:

```bash
docker stop jev-mission
```

To load code or configuration changes:

```bash
docker stop jev-mission
./scripts/mission_demo.sh
```

Refresh the dashboard afterward. Run only one mission launch at a time.
`docker start jev-mission` alone starts the container, not the application.
Dashboard HTML changes are loaded on browser refresh; Python and startup
configuration changes require relaunching.

## How the mission works

1. Sample a connected, collision-clear start and goal at least 3 metres apart.
2. Place the robot in Gazebo and globally initialize AMCL. The true start pose is
   not supplied to AMCL or Jev during a production mission.
3. Ask Jev to choose from currently available localization actions. Local
   controllers execute the action and evaluate the result before the next request.
4. When localization is stable, Jev can start Nav2 navigation to the goal.
5. While Nav2 drives, Jev reviews telemetry at a target **5 Hz**. Local safety and
   localization checks can stop motion without waiting for an API response.
6. If localization is lost, cancel navigation, request Jev recovery, evaluate it,
   and let Jev decide whether to resume. Evaluate final arrival separately using
   simulation ground truth.

There are no periodic Jev requests while a recovery action executes or is being
evaluated. During navigation, up to four reviews may be in flight; slow API calls
can reduce the achieved request rate. Five Hz corresponds to approximately 300
navigation requests per minute, plus mission decisions.

The dashboard's **owner** field identifies which controller may send motion:
`NAV` for Nav2, `RECOVERY` for recovery actions, or `NONE` for neither. Jev can
monitor Nav2 while `owner=NAV`. A command bridge enforces exclusive ownership
and expires stale commands.

| Jev action | Robot behavior |
| --- | --- |
| `SPIN` | Rotate to gather localization evidence |
| `GLOBAL_RELOCALIZE` | Reset AMCL globally, then rotate |
| `BACKTRACK_AND_SPIN` | Reverse along verified recent straight motion, then rotate |
| `MOVE_LOCAL` | Turn and drive to a safe robot-relative X/Y target within 0.5 m |
| `NAVIGATE_TO_GOAL` | Start Nav2 after stable localization |
| `REPLAN_PATH` | Stop, compute and validate a new route to the same goal, then follow it subject to safety clearance |
| `CONTINUE_NAVIGATION` | Keep the existing goal running |
| `PAUSE_NAVIGATION` | Cancel navigation and pause before reassessing |
| `RESUME_NAVIGATION` | Replan to the same destination after recovery |
| `REQUEST_HELP` | Stop and wait for operator intervention |
| `STOP` | End the mission stopped |

Available actions depend on the stage, sensor evidence, clearance, and remaining
attempts. Local movement is offered after an attempted spin when localization
still needs recovery and a safe target exists. Jev is not forced to choose it.

The action and local-target confidence threshold is **20%**. Confidence is distinct
from action probability; the dashboard displays both and highlights the largest
offered probability. Unavailable actions are labeled **Not offered**.

Spin stages finish early after **two seconds** of continuous navigation-quality
localization: fresh data, healthy status, uncertainty below **0.35**, and scan/map
agreement at least **0.85**. A HEALTHY label alone does not meet this stricter
condition. The robot then stops for the normal four-second recovery evaluation.

## Obstacle safety

All navigation and recovery commands pass through a shared 20 Hz obstacle filter.
It checks a circular robot footprint along the commanded and measured motion,
including a conservative stopping envelope. Commands slow near obstacles and stop
when the envelope is blocked, lidar coverage is invalid, or scan/odometry evidence
is stale. It uses scan-time transforms and requires full 360-degree lidar coverage.

The dashboard shows `CLEAR`, `SLOW`, `BLOCKED`, or `SENSOR_FAULT`, the reason,
envelope clearance, and requested versus applied velocity. A blocked action revokes
motion ownership and cancels navigation or aborts recovery. Jev receives safety
telemetry, but cannot override the filter. No periodic Jev calls run in `SAFETY_WAIT`. One bounded assessment may offer replanning when localization and sensor evidence are reliable.
After sustained clearance and cancellation, Jev decides the next action; buffered
commands are discarded. A blockage lasting 15 seconds requests operator help.

Tune the simulation assumptions in
`src/jev_localization_recovery/config/obstacle_safety.yaml`. The footprint radius is
0.18 m plus a 0.05 m margin. Reaction allowance is 0.15 s plus scan age; assumed
braking deceleration is 0.3 m/s². These are conservative simulation settings, not
measured hardware guarantees. The filter does not estimate obstacle velocities.
See the [mission guide](docs/mission-demo.md#obstacle-safety-and-simulation-tests)
for obstacle insertion and the live validation command.

## Test delocalization

During navigation, use the dashboard's delocalization control. It changes AMCL's
pose estimate while leaving the physical robot in place. Watch for localization
loss, navigation cancellation, a Jev recovery choice, and eventual resumption or
an operator-help request. The [mission guide](docs/mission-demo.md#inject-a-localization-error)
includes the equivalent ROS service command and parameter meanings.

## Working on the code

The repository is mounted at `/ws` in the `jev-mission` container. Its ROS domain
is `89`; build outputs use `build_mission`, `install_mission`, and `log_mission`.
Start with these files:

| File or directory | Purpose |
| --- | --- |
| `scripts/mission_demo.sh` | Docker setup, build, and launch entry point |
| `src/jev_localization_recovery/launch/jev_mission.launch.py` | Simulation, localization, navigation, and mission bringup |
| `src/jev_localization_recovery/jev_localization_recovery/mission_node.py` | Mission state machine, Jev decisions, recovery evaluation, navigation supervision |
| `src/jev_localization_recovery/jev_localization_recovery/mission_policy.py` | Available actions, bounded local targets, and mission decision schema |
| `src/jev_localization_recovery/jev_localization_recovery/recovery_selector.py` | Jev HTTP adapter and original recovery selector |
| `src/jev_localization_recovery/jev_localization_recovery/localization_monitor.py` | Localization health and uncertainty |
| `src/jev_localization_recovery/jev_localization_recovery/scan_map_score.py` | Laser-to-map scoring and clearance |
| `src/jev_localization_recovery/jev_localization_recovery/recovery_actions.py` | Rotation, global reset, and backtracking execution |
| `src/jev_localization_recovery/jev_localization_recovery/local_move.py` | Odometry-relative turn-and-drive controller |
| `src/jev_localization_recovery/jev_localization_recovery/mission_bridge.py` | Exclusive velocity ownership and command expiry |
| `src/jev_localization_recovery/jev_localization_recovery/dashboard.py` and `dashboard.html` | Dashboard server, controls, and UI |
| `src/jev_demo_interfaces/srv/InjectDelocalization.srv` | Typed delocalization service |
| `src/jev_localization_recovery/test/` | Unit and ROS runtime tests |

Configuration lives in `src/jev_localization_recovery/config/`:

- `recovery_params.yaml`: confidence, recovery, and Jev supervision settings.
- `house_amcl.yaml`: AMCL settings.
- `mission_nav2.yaml`: Nav2 planner and controller settings.
- `mission_nav.xml` and `mission_through.xml`: navigation behavior trees.
- `mission.rviz`: visualization preset.

Read the mission node and policy together before changing action availability.
Keep sensor freshness, collision checks, bounded motion, and command ownership
in the local execution layer. Do not send simulator ground truth or credentials
in Jev request bodies. After changing code, run the relevant checks below and
restart the demo to load it.

To change the house geometry, edit `scripts/generate_house.py` and run
`python3 scripts/generate_house.py` to regenerate the world and occupancy map
from the same geometry.

## Build and test

After the launcher has created the container, use the following commands to build
and test inside Humble. Stop any mission with the dashboard before running checks.

```bash
docker exec jev-mission bash -lc \
  'source /opt/ros/humble/setup.bash && colcon --log-base log_mission build --build-base build_mission --install-base install_mission --symlink-install --packages-up-to jev_localization_recovery'

docker exec jev-mission bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install_mission/setup.bash && colcon --log-base log_mission test --build-base build_mission --install-base install_mission --packages-select jev_localization_recovery --event-handlers console_direct+'

docker exec jev-mission bash -lc \
  'source /opt/ros/humble/setup.bash && colcon test-result --test-result-base build_mission --verbose'
```

For tests on a host with Python and pytest installed, without ROS:

```bash
PYTHONPATH=src/jev_localization_recovery python3 -m pytest -q src/jev_localization_recovery/test
```

ROS tests skip when their dependencies are unavailable. The latest recorded full
Humble result is **127 passed, 2 opt-in tests skipped**. See
[validation notes](docs/validation.md) for tested scenarios and remaining limits.

To run a live random mission with an injected localization failure, start the
demo idle first, then run:

```bash
docker exec jev-mission bash -lc \
  'source /opt/ros/humble/setup.bash && source /ws/install_mission/setup.bash && python3 scripts/check_mission.py --seed 42 --inject'
```

This moves the simulated robot and makes real Jev API calls. Reports and event
logs are written to `docs/mission-runs/`. A separate
`scripts/check_supervision.py` check measures navigation review frequency using
an explicit known-pose test fixture; it does not validate unknown-pose startup.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| Docker is unavailable | Run `docker info` and check that the daemon is running and your user has access |
| Key file missing or API authentication fails | Check the key-file location and environment variable selection, then relaunch |
| No Gazebo/RViz window | Check `DISPLAY` and readable `XAUTHORITY`, or launch with `--headless` |
| Dashboard unavailable | Check the launch terminal for build/startup errors and run `docker ps --filter name=jev-mission`; use port 8765 |
| Duplicate launch or stale processes | Run `docker stop jev-mission`, then rerun the launcher once |
| Jev quiet during recovery | Execution and evaluation use local checks; a new decision follows evaluation |
| Jev action rejected | Read the dashboard reason for low confidence, stale sensors, blocked targets, or exhausted attempts |
| HELP state | Inspect the cause, intervene as needed, then acknowledge through the dashboard |
| HEALTHY but still spinning | Check the stricter 0.85 agreement, 0.35 uncertainty, and two-second stable-lock requirements |

The launch terminal carries ROS logs. `/demo/state` and `/demo/events` provide
mission telemetry; the dashboard presents the same mission state and decision
history. Historical run reports record failures as well as successes.

## Current limits

AMCL can converge incorrectly in similar rooms. Stable health and scan agreement
are evidence, not proof of the true global pose. Random missions can require help
because of ambiguous localization, unavailable safe actions, or navigation failure.
Local movement and backtracking have automated coverage but have not both been
demonstrated as Jev-selected actions in a complete live mission. The latest
stable-lock spin change has automated coverage, not full random-mission validation.
This project is a simulation demo; physical robot operation has not been validated.

## Jev-controlled route replanning

`REPLAN_PATH` is offered during navigation or after an obstacle stop when
localization is ready, safety telemetry is fresh, and the planner/controller are
available. It shares the 20% confidence threshold and has a limit of two attempts
per mission. Sensor faults do not permit replanning. A blocked episode permits
one assessment after cancellation; choosing pause returns to waiting without
periodic requests. Obstacles alone do not call for global AMCL initialization.

The robot first revokes ownership and cancels its active goal. Nav2's
`ComputePathToPose` then plans while stationary using current costmaps. This action
never clears obstacle observations. The response must succeed, contain a finite,
continuous map-frame route, start near the estimate, and end near the original
goal. Planning times out after eight seconds. Invalid/no-path results request help.

A valid route does not enable motion. The bridge may discard the previous blocked
trajectory only while the owner is NONE and measured motion is near zero. It
still requires valid sensors, footprint clearance, and sustained-clearance delay.
Every new command is checked against its own stopping envelope. Safe departure
must be available within five seconds; otherwise the mission requests help.
`FollowPath` executes the exact validated replacement route, with Jev supervision.

The navigation behavior trees now compute once per goal instead of automatically
replanning every second. `RESUME_NAVIGATION` remains a restart after interruption;
`REPLAN_PATH` is an explicit route-replacement decision with its own attempt budget.
The dashboard shows its probability when offered, attempt count, result, and route.

## Live safety tuning and help acknowledgment

Click **Tune safety settings** in the control panel's Obstacle safety section to
open a separate small window, or open [Safety tuning](http://localhost:8765/safety).
It displays the active values reported by the bridge alongside editable values:

| Setting | Units | Allowed range |
| --- | --- | --- |
| Robot radius | m | 0.18 to 0.35 |
| Extra clearance margin | m | 0.02 to 0.30 |
| Assumed braking deceleration | m/s² | 0.05 to 2.0 |
| Reaction allowance | s | 0.05 to 1.0 |
| Slowdown band | m | 0.05 to 1.0 |
| Scan and odometry timeout | s | 0.10 to 0.75 |
| Sustained-clearance release delay | s | 0.20 to 5.0 |
| Blocked wait before operator help | s | 2 to 120 |

Use **Stop mission** before applying changes, or tune while IDLE, HELP, or SUCCEEDED.
The server also checks that no goal or update is pending. The bridge independently
requires fresh stopped odometry and owner NONE. Updates are applied atomically;
an invalid value rejects the whole update. Applied changes discard old commands
and trigger a fresh clearance check. **Reload active values** discards unsaved
edits in the window. Changes last only for the current session; edit
`config/obstacle_safety.yaml` for defaults used on restart.

Increasing assumed braking strength shortens the stopping envelope; increasing
the sensor timeout accepts older data. These are simulation tuning controls.
Existing recovery-specific spin/rear clearance limits remain separate YAML settings.

After **Intervention complete: reassess**, the mission now enters ACK_RECHECK and
waits for fresh localization checks before asking Jev again. Previously it reset
the stable-lock timer but immediately called Jev, temporarily hiding navigation
and creating a repeated REQUEST_HELP loop near obstacles. HELP now explicitly
requires acknowledgment instead of labeling REQUEST_HELP as an available next
choice. An obstacle, stale sensor, or genuinely unreliable localization can still
prevent motion after acknowledgment.
