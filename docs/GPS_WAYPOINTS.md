# Entering the day's GPS waypoints

The judges hand out the waypoints before the run (2026 rules p.9: the entrance
and exit of No Man's Land, and two more guiding to the ramp). They go in **one
file**, and nothing else needs to change:

```text
src/igvc_test_bringup/config/mission_planner_config.yaml
```

## What to edit

Three lists, one entry per waypoint, in the order the robot meets them:

```yaml
    waypoint_lats:  [42.6780120, 42.6781050, 42.6781900, 42.6782800]
    waypoint_lons:  [-83.1950210, -83.1949100, -83.1950300, -83.1949500]
    waypoint_kinds: ["start", "obstacle", "obstacle", "end"]
```

(The numbers above are an example, not a course.)

- **Decimal degrees**, south and west negative. Keep at least six decimals: one
  millionth of a degree is about 0.11 m of latitude.
- **`start`** is the mission entry. The robot lane-follows until a valid fix is
  within `gps_start_match_radius_m` (10 m) of it. Then the planner anchors the
  waypoints to where the robot is in odometry at that moment and stamps the
  obstacles, and the mission starts by itself (`auto_start: true`, the
  default since 2026-10-08; nobody may touch the robot during a run).
- **`obstacle`** waypoints are virtual obstacles stamped into the obstacle map,
  not goals.
- **`end`** is the Nav2 goal. When it is reached the planner hands control
  back to the lane navigator.

The shipped file holds `0.0` placeholders. The planner refuses to start with
them and says so in its log, at start-up and every 5 s:
"the start waypoint is (0, 0), the placeholder in mission_planner_config.yaml".

## After editing

- A workspace built with `--symlink-install` (the simulator containers) reads
  the file directly. Anything built without it needs
  `colcon build --packages-select igvc_test_bringup` before the run.
- Check the log of `mission_planner_node`. Before the robot reaches the start
  it prints "waiting for the start waypoint: the robot is N m away". At the
  start line, N should be the distance from the start line to the first
  waypoint. A number in the thousands means a typo or a sign error.

## Known limits (2026-10-08)

- The planner still assumes the odometry axes point east and north. On the
  real robot they point wherever it faced at start-up, so waypoints come out
  rotated by that heading. This is the next fix (Task 5c in
  `NEXT_SESSION_PROMPT.md`); until it lands, GPS waypoints are only right if
  the robot starts facing east.
- The fix is the antenna's position and the odometry is `base_link`'s, 0.22 m
  apart (`front_gps_antenna_link` at 0.177, -0.133 m). The planner does not
  correct for it; measured in the simulator as a 0.22 m error in where the
  waypoints land.

## The test

`scripts/gazebo/measurements/gps_start_check.sh`, inside the simulator
container, checks that the planner never starts on the placeholders and says
why, does not start while the robot is 8 m from the start waypoint, and does
start once the robot drives within the radius, with the obstacles where ground
truth puts them.
