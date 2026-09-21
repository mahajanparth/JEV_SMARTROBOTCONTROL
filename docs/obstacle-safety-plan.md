# Obstacle safety feature plan

Status: proposed; no behavior changes implemented.
Branch: `feature/obstacle-safety`.

## Existing protections

Nav2 uses lidar obstacle layers, inflation, and regulated pure-pursuit collision
checking. Recovery actions check spin/rear clearance; local moves check an
odometry-relative path corridor. The mission bridge is the final `/cmd_vel`
publisher for both navigation and recovery. It evaluates commands at 20 Hz,
expires commands and ownership after 0.4 seconds, and requires a recent scan
with its nearest valid finite range greater than 0.20 m.

The bridge currently has no direction-dependent swept-footprint check, gradual
slowdown, stopping-distance model, or published explanation of an obstacle veto.
Its scan reduction can overlook invalid sectors if other valid returns exist,
and does not transform obstacle points into the base frame. A fixed minimum
range also cannot distinguish an obstacle ahead from one beside or behind the
robot. The existing local protections remain useful and should be retained.

## Proposed command flow

Nav2 or recovery controller -> exclusive owner selection -> shared obstacle
safety filter -> robot `/cmd_vel`.

Keep the bridge as the sole final publisher. Put the geometry and decision logic
in a separately testable `obstacle_safety.py` module. Do not add a competing
publisher or let Jev bypass the filter.

## Implementation sequence

1. Define a configurable robot footprint and margin consistent with the Nav2
   model. Transform lidar points into `base_footprint` using fresh scan-time TF.
   Validate scan metadata, timestamps, coverage, and ranges. Treat valid positive
   infinity as clear only to the sensor's finite maximum range. Missing or invalid
   coverage required for the motion must stop that motion.
2. Predict the footprint swept by translation, reversing, rotation, and curved
   motion. Combine commanded and measured velocity with a conservative braking
   deceleration and reaction-latency allowance. Include scan age, processing,
   command transport, and control interval in the stopping envelope. Tune the
   simulation values from measured stopping behavior rather than assuming a
   fixed distance is sufficient.
3. Return CLEAR, SLOW, BLOCKED, or SENSOR_FAULT. Scale translation and rotation
   together when slowing curved motion to preserve curvature. Stop immediately
   when the stopping envelope is obstructed or required evidence is unavailable.
   Retain the steady-clock 20 Hz gate initially; measure actual response latency.
   Require sustained clearance before release to prevent stop/start oscillation.
4. Publish `/demo/safety` with status, reason, scan/TF age, relevant clearance,
   speed scale, owner, and requested versus applied velocity. Record state
   transitions without flooding the event log.
5. Make BLOCKED and SENSOR_FAULT explicit mission events. Revoke motion ownership,
   cancel Nav2 or abort recovery, and discard old buffered commands. A clear scan
   alone must not restart movement. After stable clearance and cancellation,
   request one Jev decision with fresh safety evidence. Persistent blockage or
   sensor faults lead to a bounded wait followed by operator help. Jev retains
   its existing actions, including pause, safe local move, resume, help, and stop;
   action availability must be rechecked locally. Obstacle blockage alone must
   not trigger an AMCL global reset.
6. Show a dashboard safety panel with the status, reason, clearance, requested
   and applied speed, and event history. Include a compact safety summary in Jev
   inputs. Preserve target 5 Hz navigation reviews and avoid periodic calls while
   stopped waiting for an obstacle or evaluating recovery.
7. Add a simulation-only obstacle insertion/removal control or test helper using
   Gazebo entities. Keep this separate from AMCL delocalization injection. Choose
   collision-free obstacle spawn locations and track only test-owned objects.

## Files expected to change

- New `obstacle_safety.py` and focused geometry/policy tests.
- `mission_bridge.py`: filter every selected command and publish safety state.
- `mission_node.py` and `mission_policy.py`: blocked-action handling and Jev input.
- Configuration and launch files: footprint, margins, braking, freshness, release
  delay, and blocked timeout parameters with documented units.
- `dashboard.py` / `dashboard.html`: safety telemetry and simulation controls.
- Runtime tests and a Gazebo obstacle check script.
- README, mission guide, and validation notes.

## Acceptance checks

- An obstacle in the stopping envelope blocks both Nav2 and Jev recovery commands.
- Tests cover forward, reverse, spin, curved motion, and side obstacles outside
  the swept path; a single forward cone is insufficient.
- Invalid sectors, stale scans, missing TF, stale odometry, and ownership expiry
  stop motion with a specific reason.
- Removing an obstacle does not replay buffered commands or auto-resume a goal.
- Jev cannot override a stop or execute a target newly blocked after its response.
- Repeated obstacles do not produce unbounded calls or oscillating motion.
- Gazebo validation measures command suppression latency, physical stopping
  distance, minimum footprint clearance, and contact occurrence. The existing
  random mission and delocalization scenarios must still work.

This is a simulation safety improvement, not a hardware safety certification.
A real robot would also need a tested base watchdog, braking behavior, emergency
stop, and sensing appropriate to hazards outside a 2D lidar scan plane.
