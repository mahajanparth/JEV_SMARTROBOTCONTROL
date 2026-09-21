# Verification results

The package was built with `colcon` in ROS 2 Humble, without changing host ROS or
Nav2/AMCL source. The regular tests cover scoring geometry, covariance, health,
decision selection, motion safety, backtrack history, DDS/TF transport, recovery
evaluation, cooldowns, STOP/reset, and command ownership.

Latest `colcon test` result: **90 passed, 2 opt-in tests skipped**, zero failures.
This includes the Jev response schema, HTTP adapter, asynchronous decisions,
timeouts, stale responses, sensor loss, confidence threshold, and attempt limit.
The Gazebo recovery and mission end-to-end checks were run separately.
Mission tests additionally cover bounded local targets, connected random sampling,
late Nav2 goal acceptance after STOP, help acknowledgment, typed injection, and
exclusive expiring velocity ownership.

## Gazebo TurtleBot3 house: verified

Gazebo Classic 11.10.2 with the official TurtleBot3 Burger and a custom 12 × 10 m
house. Gazebo physics, lidar, odometry, ROS simulation clock, real AMCL, and RViz
all run in `jev-gazebo`, ROS domain 88. Desktop rendering was visually checked.

The automatic check published a relative AMCL initial-pose offset of +4 m, +4 m,
and +1.5 radians, leaving the physical robot unchanged. It observed:

- Initial HEALTHY score: **1.00**.
- Injected estimate: **LOST**, score **0.282**, pose jump **5.657 m**.
- Mock decision: **GLOBAL_RELOCALIZE**, confidence **0.74**.
- A real AMCL global initialization service call and positive angular commands.
- Odometry rotation: **6.351 radians**; translation below the 0.1 m test bound.
- Final **HEALTHY**, score **1.00**, and **RECOVERY SUCCESS**.

The complete before/after state is in [gazebo-validation.json](gazebo-validation.json).
Screenshots are in [images](images/). Reproduce with `scripts/gazebo_house.sh` and
`scripts/check_gazebo_recovery.py` as documented in the root README.

## Scope of verification

The original minimal kinematic simulator verified monitoring against real AMCL,
but its global-recovery run converged to only 0.578 agreement and correctly
reported failure/STOP. It is retained as an optional test harness; that opt-in
test is not claimed to pass. The Gazebo demo is the verified end-to-end path.

Backtracking is covered by local-history, motion, and clearance tests; it has not
been demonstrated end-to-end in Gazebo. Navigation goal execution and physical
hardware were not tested.
AMCL global convergence is stochastic, and each new map needs threshold/particle
tuning. No recovery success is assumed merely because an action completed.

## Jev random mission and injected navigation failure

Seed **42** completed a random-start mission using real `jev-1.13.0` decisions.
AMCL received global initialization without the true spawn pose. The test waited
for nonzero forward commands during navigation, called the typed delocalization
service, observed sensor-triggered cancellation, Jev recovery, Jev resumption,
and goal arrival. Ground-truth goal error was **0.157 m**, localization error
**0.059 m**, and yaw error **0.012 rad**.

Evidence: [seed 42 mission check](mission-runs/check-42-1789950402935708006.json).
Earlier unsuccessful runs are retained in the same directory, including a
configuration error and a seed 7 low-confidence decision (0.49) that correctly
requested help rather than moving. The confidence threshold for that run was 0.50.
Instructions now distinguish initial global localization from localization lost
during navigation. The dashboard was rendered and visually inspected in Chrome.

Local X/Y movement and backtracking have automated controller/safety coverage;
they have not both been demonstrated as Jev-selected actions in a complete live
mission. Physical hardware is outside this verification scope. Random missions
can still require help due to ambiguous localization or navigation failure.

The initial DWB navigation configuration stalled near a doorway on seed 7 after
recovering from injected errors. That failed check is retained as
[doorway failure evidence](mission-runs/check-7-1789950717155844411.json). The mission
configuration now uses Nav2 regulated pure pursuit with explicit turn-to-path
behavior; safety thresholds and collision checking remain enabled.

Navigation readiness now requires scan/map agreement of at least **0.85**, in
addition to stable health and covariance checks. Costmaps are refreshed before
starting or resuming navigation to remove obstacle observations accumulated under
an incorrect map pose. Two additional tests cover the stricter arrival gate and
STOP during costmap refresh. The latest end-to-end rerun was interrupted by the
requested clean restart; these final changes are not yet claimed as live-validated.


## Five-Hz Jev navigation supervision

A focused live check supplied a known pose to AMCL to isolate navigation
supervision from unknown-start localization. With Nav2 driving, the real Jev
endpoint received **40 supervision requests at 4.999 Hz**, and the controller
applied **37 CONTINUE_NAVIGATION decisions** during the observation window.
The test stopped the mission afterward and motion remained disabled.

Evidence: [live supervision report](mission-runs/supervision-check.json).
This is a supervision test, not a new claim that arbitrary random-start missions
will localize successfully. The runtime uses a 0.2-second target interval, at most
four concurrent requests, and discards superseded or cancelled reviews. Tests cover
concurrency bounds, response ordering, stale replies after STOP, intervention
cancellation, timeout/error stops, and specific confidence rejection messages.

## Stable-lock spin completion and 20% confidence

The current action and local-target confidence threshold is 0.20. Runtime tests
cover rejection at 0.19 and acceptance at 0.20 and 0.31, without reporting rejected
local targets as executed actions. Spin stages of all three rotation recoveries
now finish after two seconds of stable navigation-quality localization; tests
also verify that brief or weak locks do not finish a spin. These changes passed
the regular suite above; they have not yet been validated in a full random mission.
