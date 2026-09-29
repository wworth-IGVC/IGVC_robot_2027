# ROS 2 Jazzy: every package and piece of robot software, and what it needs

**Written 2026-09-24 and finished 2026-09-29** on branch `jazzy-only`, to answer Aidan's question of
2026-09-22: of all the packages and software needed to run the robot, how much
exists for Jazzy, what needs migrating, and what needs rewriting.

**The short answer:** all 22 ROS packages in this repository build on Jazzy
with no code change, and **nothing needs rewriting because of Jazzy**. What
needed migrating in the repository was packaging (packages that used things
they never declared) and one image recipe (the torch image); both are done on
this branch. What still needs migrating is on the robot: the Jetson's OS, the
ZED SDK and wrapper, and torch on the Jetson. Those need hardware and your
decisions, and section 5 lays them out.

How to read this page:

| Mark | Meaning |
| --- | --- |
| **MEASURED** | a command ran and the result is recorded (section 8 says where) |
| **INFERRED** | reasoned from reading the code or a config, not run |
| **WEB** | from a vendor page or release, cited with its date, **not checked on hardware** |
| Used by | `robot` (a real-robot launch path starts or loads it), `sim` (the Gazebo path), `both`, `Isaac only`, `manual tool` (run by hand, no launch file), `unused` |

Laptop numbers are never Jetson numbers. Everything measured here ran on an
x86 laptop (RTX 5070 Ti) in Docker; nothing ran on the Orin.

---

## 1. Summary

| Verdict | Count | Packages |
| --- | --- | --- |
| **Runs on Jazzy** (already runs in a Jazzy container) | 3 | `igvc_lane_detection`, `igvc_test_description`, `grr_hardware` |
| **Jazzy binary exists** (an apt package; our vendored copy could go) | 4 | `ublox_gps`, `ublox_msgs`, `ublox_serialization`, `zed_description` |
| **Builds unchanged** (compiles and installs on Jazzy, not run) | 9 | `debug_gui`, `igvc_lidar_test`, `igvc_simulation_interface`, `odrive_ros2_control`, `odrive_can`, `sllidar_ros2`, `yolo_ros`, `yolo_msgs`, `yolo_bringup` |
| **Migrate** (a named change; both done on this branch) | 2 | `igvc_test_bringup` (packaging), `ping_location` (one dependency) |
| **Rewrite** | **0** | |
| **Drop** (nothing uses it) | 4 | `greenwave_monitor`, `greenwave_monitor_interfaces`, `ublox` (metapackage), `odrive_botwheel_explorer` |

What that rests on, all MEASURED in clean Jazzy containers on 2026-09-24:

- **22 of 22 packages build, four times over:** in `ros:jazzy-ros-base`
  (4 min 49 s), on the Gazebo image plus 41 apt packages (5 min 49 s), and on
  this branch's final manifests without `--symlink-install`, both in a clean
  `ros:jazzy-ros-base` (10 min 25 s, run alongside another build) and in
  `igvc-dev-jazzy` (6 min 21 s), 2026-09-29. 0 failed; warnings only from
  `odrive_ros2_control` and `sllidar_ros2`; 0 undefined symbols in 48 shared
  libraries.
- **Every Python module imports in `igvc-dev-jazzy`: 36 of 36** (2026-09-29).
  On a plain Jazzy install 32 of 36 import now (31 before this branch); the 4
  that do not need pip-only packages by design (torch, ultralytics twice,
  imutils), which is what the dev image adds.
- **A clean `rosdep install` now brings in what the packages use**, including
  `twist_stamper`: strict `rosdep install` (no `-r`) exit 0 and
  `rosdep check` exit 0 in a clean `ros:jazzy-ros-base`, and `rosdep check`
  exit 0 in the dev image (2026-09-29). Before this branch it exited 2, and a
  clean install left the real robot without `twist_stamper`, `joy` and
  `teleop_twist_joy`.

### The work that is left, in order

| # | What | Who decides | Section |
| --- | --- | --- | --- |
| 1 | JetPack on the Orin Nano Super: 7.2.1 (Ubuntu 24.04, native Jazzy) or 6.2.x (Jazzy only in containers) | Aidan | 5 |
| 2 | Capture card: only the ZED Link **Duo** supports the Orin Nano (the Quad does not) | Aidan | 5 |
| 3 | ZED SDK 5.5 with wrapper v5.5.0, built from source on Jazzy, proven by launching the node | robot team, after 1 | 5 |
| 4 | torch on the Orin (Python 3.12), proven with a real kernel on `sm_87` | robot team, after 1 | 5 |
| 5 | On the robot: bring up `can0` before `ros2_control_node`; check the GPIO nodes are not in mock mode | whoever brings the robot up | 6 |
| 6 | Bring in the competition branch or not; then re-baseline the simulator | Aidan | 4 |

---

## 2. Every package in `src/`

One row per `package.xml`, 22 in all. `python3 scripts/check_jazzy_inventory.py`
fails if a package is missing from this table, listed twice, or listed after
it has been removed from `src/`.

<!-- package-table:start -->
| Package | Kind | Used by | On Jazzy | Verdict | Effort |
| --- | --- | --- | --- | --- | --- |
| `debug_gui` | team | Isaac only: `lane_compare_ui` from `isaac_lane_test.launch.py:207-217` | builds; 3 of 4 modules import on plain Jazzy, all 4 in the dev image (M) | Builds unchanged | none |
| `greenwave_monitor` | vendored submodule | unused: only `.gitmodules:22-24` names it, on `main` and on both 2026 competition branches | builds (M); `ros-jazzy-greenwave-monitor` 1.0.0-1 on amd64 and arm64 (M) | Drop | S |
| `greenwave_monitor_interfaces` | vendored submodule | unused (only `greenwave_monitor` depends on it) | builds (M); binary 1.0.0-1 (M) | Drop | none |
| `grr_hardware` | team submodule | Isaac only: `IsaacDriveHardware`. The robot's `CanInterface` maps to the ODrive plugin (`ros2_control_info.urdf.xacro:10-16`) | builds with no warnings; loaded, activated and driven by Jazzy's controller manager 4.48 (M) | Runs on Jazzy | none |
| `igvc_lane_detection` | team, protected | both: robot `lane_segmentation_node`, `navigation_node`, `odom_tf_bridge_node`, GPIO nodes; sim ground truth, navigator, localization | builds; 17 of 17 modules import (M); 5 nodes have run in the Jazzy simulator (M); YOLOPv2 ran in `igvc-dev-jazzy` (M, section 7) | Runs on Jazzy | S |
| `igvc_lidar_test` | team | manual tool: no launch file or script starts it | builds, imports (M) | Builds unchanged | none |
| `igvc_simulation_interface` | team | Isaac only: `simulation_interface.launch.yaml:39-42` | builds, imports against `simulation_interfaces` 1.5.1 (M); not run against Isaac on Jazzy | Builds unchanged | none |
| `igvc_test_bringup` | team | both: `igvc_fused_drive.launch.py` is the real robot's top level; `gazebo_sim` and `gazebo_nav_test` are the simulator | builds; the sim launch files run (M); packaging broken until this branch (M) | Migrate: packaging, done here | S |
| `igvc_test_description` | team | both: the robot URDF, meshes and the Gazebo world | builds; xacro expands for both hardware options and for `sim:=true` (M) | Runs on Jazzy | none |
| `odrive_botwheel_explorer` | vendored (org fork of `ros_odrive`) | unused: upstream's demo robot | builds (M); its `cmd_vel_unstamped` remap targets a topic the controller does not have | Drop | none |
| `odrive_can` | vendored (org fork of `ros_odrive`) | unused: the robot uses the ros2_control plugin | builds clean; already carries its own Jazzy guard (`odrive_can_node.cpp:48-56`) | Builds unchanged | none |
| `odrive_ros2_control` | vendored (org fork, two team commits) | robot: the default `hardware_interface:=CanInterface` | builds with one deprecation warning; loads and initialises; stops without `can0` (M) | Builds unchanged | S |
| `ping_location` | team | unused: no launch file, script or subscriber | builds; imports only once `python3-requests` is installed (M) | Migrate: one dependency, done here | S |
| `sllidar_ros2` | vendored submodule | robot sensor, run by hand on `main` (the competition branch started it from `sensor_launch`) | builds with vendor-SDK warnings (M); **no Jazzy binary** on amd64 or arm64 (M) | Builds unchanged | S |
| `ublox` | vendored submodule | unused metapackage | builds (M); binary 2.3.0-4 (M) | Drop | none |
| `ublox_gps` | vendored submodule | robot: `igvc_fused_drive.launch.py:96-107` | builds (M); `ros-jazzy-ublox-gps` 2.3.0-4 on amd64 and arm64, and our C++ is identical to it (M) | Jazzy binary | S |
| `ublox_msgs` | vendored submodule | robot, through `ublox_gps` | builds (M); binary 2.3.0-4 with identical messages (M) | Jazzy binary | none |
| `ublox_serialization` | vendored submodule | robot, build time only (header-only) | builds (M); binary 2.3.0-4, identical headers (M) | Jazzy binary | none |
| `yolo_bringup` | team fork submodule | unused on `main` | builds (M) | Builds unchanged | none |
| `yolo_msgs` | team fork submodule | unused on `main` | builds (M) | Builds unchanged | none |
| `yolo_ros` | team fork submodule | unused on `main`; the competition branch's `obstacle_costmap` would consume it, commented out there | builds (M); imports in `igvc-dev-jazzy`, which has torch and ultralytics (M) | Builds unchanged | M |
| `zed_description` | vendored submodule (Stereolabs, tag 0.1.5) | both: the camera macro every robot expansion includes | builds (M); `ros-jazzy-zed-description` 0.1.5-1, the same version (M) | Jazzy binary | S |
<!-- package-table:end -->

### 2.1 Notes by package, where a row needs more than a line

**`igvc_test_bringup`, the one that mattered.** Three packaging defects,
MEASURED on 2026-09-24 and fixed on this branch:

1. `package.xml:10` declared `<buildtool_depend>ament_python</buildtool_depend>`,
   which is not a rosdep key. `rosdep install` without `-r` failed on it and
   `rosdep check` exited 2.
2. Every `exec_depend` sat inside one XML comment, so the package declared no
   runtime dependency at all. A clean `rosdep install` then did not install
   `twist_stamper`, and **on Jazzy the real robot does not move without it**
   (section 6.1). `joy` and `teleop_twist_joy` were never in the commented
   block either, so uncommenting it alone would not have fixed the joystick.
3. `setup.cfg` had `[develop] script_dir` but no `[install] install_scripts`,
   so a normal (non-symlink) build put `gazebo_odom_shim` and
   `sim_startup_helper` in `bin/`, where `ros2 run` and launch files do not
   look. Both simulator routes build with `--symlink-install`, which hid it.

It now declares 29 runtime dependencies, grouped by the launch path that
uses them. `zed_wrapper` is deliberately not declared: it has no Jazzy binary,
so a key for it would make every `rosdep install` on noble fail. It comes from
the ZED image instead.

**A mistake made and fixed on the way.** The first version declared 34,
including seven packages this workspace builds from source (`grr_hardware`,
`odrive_ros2_control`, the three u-blox packages, `debug_gui`,
`igvc_simulation_interface`). rosdep ignores workspace packages, so they
installed nothing. But colcon then required all seven to be built before
`igvc_test_bringup`, even under `--packages-select`, and the simulator's
four-package build failed with "Failed to find .../package.sh" (MEASURED, the
first smoke test in the dev image). They are now a comment in `package.xml`
saying why. The two workspace packages it still declares,
`igvc_test_description` and `igvc_lane_detection`, are both in the
simulator's build.

**`igvc_lane_detection`** declared `tf2_ros` (the C++ package) but six nodes
import the Python `tf2_ros` module, which ships in `tf2_ros_py`; on Jazzy it
arrived only through `tf2_geometry_msgs`. `track_ground_truth_node.py:57`
imports `yaml`. Both are now declared (a `Jazzy compat:` commit, since this
package is on the protected list). Its torch, TensorRT, ultralytics and
`Jetson.GPIO` imports are all guarded or lazy, so every module imports
without them; section 6.3 says why they are not declared and how they are
installed instead. **Importing is not running, though:**
`lane_segmentation_node` loads its model in the constructor, and the loader
raises when torch is missing (`lane_segmentation.py:289`,
`yolopv2_infer.py:150-156`), so without torch that node exits at startup and
the robot's default launch has no lane perception (INFERRED from the code, not
run without torch).

**`odrive_ros2_control`** is the org fork `Gold-Rush-Robotics/flipsky_odesc_ros_odrive`
at `800fc4b`: upstream `6386bf7` plus two team commits (encoder estimates
polled over RTR for the ODESC's 0.5 firmware; every axis rebooted and its
errors cleared on activate). Upstream says it is not for ODrive 3.x-class
boards, so **those two commits are what make it work on the Flipsky ODESC**
(INFERRED) and switching to upstream would lose them. Upstream CI builds
Humble, Iron, Jazzy and Kilted. One deprecation warning, at
`odrive_hardware_interface.cpp:112`; the Jazzy API it should move to is
`on_init(const HardwareComponentInterfaceParams&)`. That change belongs in the
org fork, not here.

**`sllidar_ros2`**: keep building it from source. There is no Jazzy binary on
either architecture (MEASURED). The `ros-jazzy-rplidar-ros` binary (2.1.0) is
**not** an alternative for the C1: its release predates C1 support, which
upstream lists under "Forthcoming" (WEB, ros2-gbp release tag, accessed
2026-09-24). This corrects the 2026-09-24 inventory, which offered that switch.

**`ublox_gps` and friends**: the binary is the same code, so the vendored copy
could be replaced by `apt install ros-jazzy-ublox-gps` after one GPS bench
test. Not done here: the submodule is harmless and the switch wants hardware.

**`zed_description`**: either works. The source copy keeps the pixi route
independent of whether RoboStack packages it.

**The four drops** can be skipped at build time with `colcon build
--packages-skip ...`; removing a submodule is a change to the parent
repository and Aidan's call.

---

## 3. What the build and dependency checks found beyond `src/`

- **Gazebo arrives in every rosdep-built image, robot included**, through
  `nav2_bringup`, which on Jazzy depends on the TurtleBot simulation packages
  (`nav2_minimal_tb3_sim`, `nav2_minimal_tb4_sim`) and so on `ros_gz_sim` and
  `gz_sim_vendor`. MEASURED: a clean install with `igvc_test_bringup`
  declaring nothing still installed both. So declaring `ros_gz_sim` changed
  nothing for the robot, and keeping Gazebo off a Jetson means not
  rosdep-installing `nav2_bringup`, a separate choice.
- **`EventsExecutor` exists on Jazzy** (`rclpy` 7.1.11, MEASURED), so
  `lane_segmentation_node`, `navigation_node` and `odom_tf_bridge_node` run
  under it on Jazzy; on Humble they fell back to `SingleThreadedExecutor`.
  The competition container was Jazzy, so this is what competed.
- **The competition container was already Jazzy**: its scripts source
  `/opt/ros/jazzy` and use `python3.12` (MEASURED from the script text on
  `2026/more_diverging_changes`). So the drive, lidar, GPS and Nav2 code most
  likely already ran on Jazzy on the robot (INFERRED). The remaining robot
  questions are JetPack, the ZED stack and the Orin Nano, not a distro port.

---

## 4. What the competition branch adds

`2026/more_diverging_changes` (`67a6934`, 61 commits never merged to `main`)
is Jazzy code. It merges into this fork with **one trivial conflict** (the
Gazebo include at the end of `test_robot.urdf.xacro`) plus two fixups in the
merge commit: keep `localization.py` and its entry point (the simulator
launches `localization_node`, which the branch deletes), and keep `main`'s
five submodule pointers (the branch's point backwards, and
`IGVC_track_generator`'s would lose `track_points.json`). Bringing it in is
Aidan's call, and it changes navigator behaviour, so the simulator gates must
be re-baselined after.

**The pointers change silently, with no conflict** (MEASURED with
`git ls-tree`): the merge base pins the same commits as `main`, so git takes
the branch's side of all five without asking. Four of them are older
ancestors of `main`'s and should be reset. **`isaac/exts` is the exception**:
the branch pins `72ff9c21` in the org's fork, which can find the three
URDF-imported cameras, where `main`'s Stereolabs `375eddd` cannot, but which
is 1 commit ahead of `375eddd` and 12 behind it. Decide it deliberately; `BRANCHES.md` has the
25-line alternative.

**To look at the branch running, without merging anything:**
[`COMPETITION_STACK.md`](COMPETITION_STACK.md) builds `67a6934` in a
container and opens a ROS 2 terminal on it, all from PowerShell.

| Piece | Ran at competition? | On Jazzy | Verdict if adopted |
| --- | --- | --- | --- |
| `lane_segmentation.py`, rewritten: YOLOPv2 TorchScript, front camera, projected through the ZED organised point cloud | **yes**, the lane detector that competed | Python, imports (INFERRED from `main`'s version, MEASURED); needs torch, the weights and, in sim, the point-cloud frame fix | Runs on Jazzy |
| `lidar_obstacle_costmap.py`: LaserScan to `/lidar_obstacle_map`, read by the navigator | **yes**, the only obstacle input | inputs already exist in the simulator | Runs on Jazzy |
| `navigator.py` changes: mission gate, lidar overlay, stuck-reverse, heading limit | **yes** | Python | Runs on Jazzy; changes behaviour, re-baseline |
| `mission_planner.py`: GPS waypoints, ramp | **yes**, with placeholder waypoints | needs `/fix`, an IMU rename and a ramp in the world | Builds unchanged |
| `nav2_gps_waypoint.py` | no (a separate GPS test script) | **Jazzy only** (`followGpsWaypoints`) | Builds unchanged |
| `igvc_pointcloud_tools` (new C++ package, PCL) | no, commented out | Jazzy only (`Clock::now()` const); needs `libpcl-dev` | Drop, or reference |
| `igvc_imu_interface` (new, L3GD20 gyro over I2C) | no, commented out | builds (INFERRED) | Drop |
| `depth_obstacle_costmap.py`, `obstacle_costmap.py` (yolo_ros) | no | Python | Reference |
| Anchor3DLane, UFLDv2, TensorRT backend | no | heavy CUDA builds | Reference |
| `igvc_task_runner`, `igvc_task_gui` (on `2026/full_self_driving` only) | no | builds (INFERRED) | Reference |
| `controllers.yaml`: `wheel_radius: 0.1016` | yes | config | Aidan confirms (see 6.4) |
| its `igvc_test_bringup/package.xml` declares `zed_wrapper` | | **breaks a clean rosdep install on noble** | fix in the merge |

---

## 5. Robot-side software

The robot is a Jetson Orin Nano Super 8 GB with three ZED X cameras, an
RPLIDAR C1, a u-blox ZED-F9P and ODrive-class motor controllers (Flipsky
ODESC) on CAN. Web sources were re-checked on 2026-09-24 by a second agent
against the primary pages; dates are as the source shows them.

| Component | Where used | Jazzy status | Verdict | What we must do | Effort |
| --- | --- | --- | --- | --- | --- |
| **JetPack on the Orin Nano Super** | robot OS | JetPack **7.2.1** (Jetson Linux 39.2.1, 2026-08-11) is Ubuntu 24.04 and supports the Orin family, so Jazzy runs natively; JetPack 6.x stays on 22.04, Jazzy only in containers (WEB: developer.nvidia.com/embedded/jetpack/downloads) | **Decision** | choose 7.2.1 or 6.2.x; the Orin Nano devkit flashes from a USB ISO, there is no SD image for 7.2 | L |
| **ZED Link capture card** | robot hardware | the Duo supports the Orin Nano; the **Quad does not** (the 2026 robot was an AGX Orin, where a Quad works). Three cameras on a Duo = 2 + 1; the paired group is capped at 30 fps (WEB: docs.stereolabs.com, ZED Link compatibility, accessed 2026-09-24) | **Decision** | confirm which card we own; buy a Duo if it is a Quad | M |
| **ZED SDK** | robot, the camera container | 5.5.0, 2026-09-17: installers for JetPack 7.2, 6.2.2, 6.1/6.2 and x86 Ubuntu 22/24/26 (WEB: stereolabs.com/developers/release) | Migrate | move to 5.5 paired with wrapper 5.5 | M |
| **zed-ros2-wrapper** | robot | officially supports Jazzy since `humble-v5.0.0` (2025-09-11); latest v5.5.0, 2026-09-18; accepts SDK 4.2 to 5.5 and **exits at startup** outside that (WEB: GitHub releases, `sl_version.hpp`). **No Jazzy binary** on either architecture (MEASURED) | Builds from source | pin upstream v5.5.0 and drop the org fork, which caps the SDK at 5.2; **verify by launching the node, never by building it** | M |
| **torch on the Orin** | robot, `lane_segmentation_node` | JetPack 7.2: `download.pytorch.org/whl/cu132` has torch 2.14.0 for cp312 aarch64. JetPack 6: the Jetson AI Lab index's only cp312 torch is 2.8.0 (WEB, accessed 2026-09-24) | Decision, follows JetPack | prove it with a numerical test on `sm_87`, not `is_available()` | M |
| **Jetson.GPIO** | robot: `autonomous_indicator_node` (safety light), `brake_control_node` | not an apt package on noble; comes with JetPack. **Both nodes silently fall back to a mock without it**, so the light and the brake solenoid do nothing (INFERRED from `autonomous_indicator.py:27-30`, `brake_control.py:55-58`) | Migrate (environment) | install it on the robot image and check both nodes' startup log for mock mode | S |
| **TensorRT and cuda-python** | robot, the optional TRT backend (`yolopv2_trt.py`), not the default | JetPack provides TensorRT. `from cuda import cudart` (`yolopv2_trt.py:40`) is a module removed in cuda-bindings 13 (WEB) | Migrate, only if used | check the import on JetPack 7 before choosing the TRT backend | S |
| **Nav2** | both | 1.3.13 binaries on amd64 and arm64 (MEASURED); `nav2_lane_follow_config.yaml` is already Jazzy syntax (`::` plugin names, string polygons, `costmap_update_timeout`) | Jazzy binary | nothing | none |
| **robot_localization** | robot (`zed_multi_fused_odom`, commented out of the fused drive today) | 3.8.3 binaries on amd64 and arm64 (MEASURED) | Jazzy binary | declared on this branch | none |
| **u-blox ZED-F9P** | robot | `ros-jazzy-ublox-gps` 2.3.0-4, amd64 and arm64, identical to our copy (MEASURED) | Jazzy binary | optional switch after a bench test | S |
| **RPLIDAR C1** | robot | `sllidar_ros2` builds from source; no binary; `rplidar_ros` 2.1.0 predates C1 support (MEASURED and WEB) | Builds from source | keep `sllidar_ros2`; test on arm64 | S |
| **ODrive over CAN** | robot | the org fork builds on Jazzy; upstream CI includes Jazzy (MEASURED, fork history) | Builds from source | bring `can0` up before `ros2_control_node` (6.2) | S |
| **ros2_control, `diff_drive_controller`** | robot | 4.48 / 4.42.1 binaries; `cmd_vel` is TwistStamped only (MEASURED) | Jazzy binary | `twist_stamper` declared on this branch (6.1) | none |
| **twist_stamper, joy, teleop_twist_joy** | robot | binaries on both architectures; the joystick config already publishes stamped (`xbox-holonomic.config.yaml:4`) | Jazzy binary | declared on this branch | none |
| **torch, torchvision on x86** | the simulator's perception, development | torch 2.14.0 + torchvision 0.29.0 for CUDA 13.0, cp312, `sm_120` in the arch list, a convolution ran on the RTX 5070 Ti (MEASURED) | Migrate: **done**, `igvc-dev-jazzy` | nothing | none |
| **ultralytics** | `yolo_ros`, `multi_camera_lane_detection` | 8.4.162 on Python 3.12 (MEASURED); it releases almost daily, so it is pinned | Migrate: **done**, pinned in `igvc-dev-jazzy` | reconcile with `yolo_ros`'s own pin (8.4.6) if `yolo_ros` is adopted | S |
| **YOLOPv2 weights** | robot and sim | release `V0.0.1`, 156,380,200 bytes, SHA256 `f2a8c837...627f1f8c`, now checked by the fetch script (MEASURED) | exists | nothing | none |
| **yolo_ros upstream** | unused on `main` | upstream CI builds Jazzy (WEB) | Decision | keep for the v4 obstacle-detection work, or drop | M |
| **Isaac ROS** | nothing | 4.6.0 (2026-08-18) added Orin and JetPack 7.2 on Jazzy; 5.0.0 (2026-09-21) moved to Lyrical, so 4.6 is likely the last Jazzy line (WEB; "last" INFERRED) | Drop (unused) | nothing unless a 2027 feature needs it | none |
| **The middleware (RMW)** | robot and sim | `ros-jazzy-rmw-zenoh-cpp` 0.2.10 has a noble binary (MEASURED, amd64, 2026-09-29). But three different settings are in the repo and **which one competed is unknown**: `docker-compose.jetson.yml:31-32` sets `rmw_zenoh_cpp` with a peer-mode config listening on `tcp/localhost:7447` (`zenoh_config.json5:11,23`), and **nothing in the repo starts a Zenoh router** (`git grep zenohd` finds nothing); the x86 compose files and the simulator use Fast DDS over UDPv4; the competition branch's `use_igvc_dds.sh` switches to Fast DDS shared memory only, but its compose file **executes** that script instead of sourcing it (`docker-compose.yml:38` there), so its exports never reach the launch, and the five `dazzling_easley` scripts do not source it either (MEASURED by reading) | **Decision** | pick one RMW for the 2027 robot; if Zenoh, something must start `rmw_zenohd`; then prove two containers see each other | S |
| **Isaac Sim and its ROS bridge** | Isaac only | not assessed in this pass; the Isaac track is Aidan's | out of scope | | |

**How the cameras ran in 2026, corrected 2026-09-29.** An earlier reading had
the three ZED X running in the Humble `jetson-zed` container. On the
competition branch they did not: `scripts/sensor_launch.sh` runs
`docker exec` into the same `dazzling_easley` container as the drive stack,
sources `/opt/ros/jazzy`, and starts `sensor_launch.launch.py`, which
includes `zed_wrapper`'s `zed_camera.launch.py` once per camera with models
`[zedx,zedx,zedx]` (`sensor_launch.launch.py:162-166, 219-223, 321-325` on
`67a6934`). The `jetson-zed` service's command in `docker-compose.jetson.yml`
is commented out (MEASURED by reading both). So **the 2026 robot most likely
already ran the ZED SDK and wrapper under Jazzy, in a container with a 24.04
userland on a JetPack 6 host** (INFERRED: that the robot ran these scripts is
not confirmed). If so, the "Jazzy only in containers" JetPack option is what
competed, not an untested risk, and **`dazzling_easley`'s image is the most
valuable thing on the AGX Orin**: its ZED SDK version, wrapper version,
`/opt/venv` torch build and RMW are recorded nowhere. Before anyone reflashes
that Jetson, capture it: `docker inspect dazzling_easley`,
`docker image history` of its image, `pip list` inside `/opt/venv`, then
`docker commit` and `docker save`.

**Not covered anywhere yet:** whether three ZED X at 1080p30, YOLOPv2 in FP16
and Nav2 fit on an Orin Nano Super 8 GB (every runtime number here is from an
x86 laptop), and whether any sensor or the ODrive has run on Jazzy against
real hardware (the 2027 team has not: the ODrive plugin was only loaded, and
stopped without `can0`).

---

## 6. Robot-side work this branch documents but does not do

These change, or could change, how the real robot behaves, so they are
written down for Aidan and David rather than made.

### 6.1 The velocity hop to the wheels needs `twist_stamper`

Nav2's chain ends at `/diff_drive_controller/cmd_vel_unstamped`
(`nav2_lane_follow_config.yaml:133`); `twist_stamper`
(`igvc_fused_drive.launch.py:109-122`) turns that into the TwistStamped the
controller takes on `/diff_drive_controller/cmd_vel`. MEASURED on Jazzy with
the repo's own xacro and `controllers.yaml`: a Twist on `cmd_vel_unstamped`
gives **0.0 rad/s** at the wheel without `twist_stamper` and **2.708 rad/s**
with it, which matches (0.5 + 0.2 x 0.5027 / 2) / 0.2032.

**This hop is not new with Jazzy.** On Humble, `controllers.yaml:54`
`use_stamped_vel: true` already made the controller take TwistStamped, so
`twist_stamper` was needed there too; the Humble image simply installed it.
What Jazzy changes is that `use_stamped_vel` no longer exists (MEASURED:
"Parameter not set"), so stamped is the only option. The real gap was that
nothing declared `twist_stamper`. It is declared now, which changes what
rosdep installs and not which nodes run.

**The simulator never exercises this hop**: `gazebo_nav_test_nav2_overrides.yaml:49`
sends the last hop straight to `/cmd_vel`, which Gazebo's DiffDrive takes. A
passing simulator says nothing about it.

The Jazzy-native alternative is Nav2's `enable_stamped_cmd_vel: true` on
every node that publishes or subscribes the velocity (default false on Jazzy,
flipped in Kilted; WEB, Nav2 notice 2024-12-03). It removes `twist_stamper`
but changes the robot's command path, so it is Aidan's call.

### 6.2 `can0` must be up before `ros2_control_node`

Jazzy's controller manager **exits** if a hardware component cannot be
activated at startup. MEASURED with the ODrive plugin and no CAN interface:
"Failed to initialize SocketCAN on can0", then the process terminates. Bring
`can0` up first, every time.

### 6.3 Python dependencies with no usable apt package on Jazzy

| Package | Used by | rosdep on noble (MEASURED) | How it is installed |
| --- | --- | --- | --- |
| torch, torchvision | `yolopv2_infer.py:28` (guarded), `yolo_node.py:29` | the rule `python3-torch` exists but apt has **no candidate** for it on noble amd64, so declaring it would make every strict `rosdep install` fail | pip, CUDA 13.0 wheels, in `igvc-dev-jazzy` |
| ultralytics | `multi_camera_lane_detection.py:70` (lazy), `yolo_ros` | no apt rule; `python3-ultralytics-pip` resolves to pip | pip, pinned 8.4.162 |
| lap | `yolo_ros` tracker | no rule | pip, pinned 0.5.12 |
| imutils | `debug_gui/camera_test.py:4` (not an entry point) | no apt rule; a pip rule exists | pip, pinned 0.5.4 |
| tensorrt, cuda-python | `yolopv2_trt.py:35,40` (guarded, not the default backend) | no rule | JetPack on the robot; not in the dev image |
| Jetson.GPIO | `autonomous_indicator.py:27`, `brake_control.py:55` (guarded) | no rule | JetPack on the robot only |

Pip-installer rosdep keys are not used either: rosdep would install the
newest version of each, and the newest numpy (2.x) and OpenCV (5.x) both break
the apt OpenCV and `cv_bridge` that ROS loads (MEASURED, section 7). The
image pins them behind a constraints file instead.

### 6.4 Not Jazzy, but found on the way

- `controllers.yaml:54` `use_stamped_vel` and `:51` `publish_odom` do nothing
  on Jazzy (the first no longer exists; the second never was a
  `diff_drive_controller` parameter). Left in place; harmless.
- `motor_controllers.launch.py:52-53`: Jazzy's controller manager reads the
  URDF only from `/robot_description`, so the parameter and the remap are
  no-ops (MEASURED). Harmless because `robot_state_publisher` runs.
  Wrapping the URDF in `ParameterValue(..., value_type=str)`, as the Gazebo
  launch does, is recommended but was not tested through `ros2 launch`.
- `wheel_radius: 0.2032` is the wheel's diameter by geometry; the
  competition branch sets 0.1016. `wheel_separation` 0.5027 against 0.576 from
  the model. Aidan's call; the simulator overrides both.
- The ZED wrapper's real-robot config fuses the u-blox fix
  (`common_stereo_real.yaml:156-158`), and the live drive path's
  `odom_tf_bridge_node` reads the front ZED's odometry, so **the robot cannot
  drive without the ZED container running**.
- `zed_multi.launch.py:235` and `zed_multi_real.launch.py:47` list the three
  camera serials in different orders; front and right look swapped between
  them (INFERRED; check against the cameras).
- `yolo_ros`'s `detect_3d_node` imports the Python `tf2_ros` without
  declaring `tf2_ros_py`: a submodule, so a note for its repository.
- `docker/Dockerfile.gazebo-jazzy`'s comment on why `twist_stamper` is
  installed had the direction backwards; corrected on this branch (comment
  only, the image is unchanged).

### 6.5 Found in the competition stack's own graph, measured 2026-09-29

First read from the code on 2026-09-28 and 29, then measured on the running
competition stack (`67a6934`, the 34-node graph, [`COMPETITION_STACK.md`](COMPETITION_STACK.md))
with `ros2 topic info` and `ros2 param get`. None is a Jazzy problem; each is
a question about how the robot is meant to behave, so each is Aidan's call.

| Finding | Measured | Why it matters |
| --- | --- | --- |
| **Four nodes publish `/cmd_vel_nav`, nothing arbitrates** | `Publisher count: 7`: `mission_planner_node`, `igvc_navigator`, `controller_server`, and `behavior_server` with four; 1 subscriber, `velocity_smoother`. No `twist_mux` | whichever published last wins; the ramp logic, the navigator's stuck-reverse and Nav2 can fight |
| **The robot cannot reverse** | `velocity_smoother` `min_velocity` = `[0.0, 0.0, -1.5]` | the smoother clamps the navigator's -0.30 m/s stuck-reverse to 0 |
| **Nothing publishes `/odom` on the robot** | `/odom`: `Publisher count: 0`, `Subscription count: 2`; `bt_navigator` `odom_topic` = `/odom`, `controller_server` and `velocity_smoother` = `odom` | odometry exists on the front ZED's `odom` and `/diff_drive_controller/odom`, not where Nav2 reads; the simulator hides this, because `gazebo_odom_shim` publishes `/odom`. What it does to the controller was not measured |
| **Nothing sensor-driven can stop the robot** | `collision_monitor`: `lidar_scan.enabled` and `front_pointcloud.enabled` both `False`; `StopZone.enabled` `False` | it passes every command through. Same on `main` |
| **The GPS mission cannot start as committed** | `mission_planner_node` `auto_start` `False`; `waypoint_lats` `[0.0, 0.0, 0.0, 0.0]` | the ramp logic is live, the GPS part is not |
| `diff_drive_controller` publishes no TF | `enable_odom_tf` `False` | so it cannot conflict with `odom_tf_bridge`; recorded because it was asked |
| The competition wheel values | `wheel_radius` 0.1016, `left_wheel_radius_multiplier` -1.0, `cmd_vel_timeout` 0.3 | 0.1016 is the geometry's radius; `main` still says 0.2032 (6.4) |

**Still open, not measurable here:** whether the front ZED's odometry has
`base_link` or the camera link as its child frame. If it is the camera,
`odom_tf_bridge` puts `base_link` about 0.4 m forward of the truth. That needs
a real ZED or a competition bag.

---

## 7. The dev image: what replaced the Humble torch image

`docker/Dockerfile.dev-jazzy` builds `igvc-dev-jazzy`: the published Gazebo
image, pinned by tag and digest, plus torch, ultralytics and every apt package
the workspace declares. It replaces `Dockerfile.humble-fused-drive`. Its
build fails, rather than shipping, if any of these is false:

- OpenCV is the pinned pip 4.11.0 under `/usr/local`, numpy is 1.x, torch is
  `2.14.0+cu130` with `sm_120` compiled in, ultralytics is the pinned
  8.4.162, setuptools is below 80;
- no apt-owned file under `/usr/lib/python3` has been deleted by pip;
- `cv_bridge` round-trips a colour image and a float depth image.

MEASURED results, 2026-09-24 and 2026-09-29 (report section 20):

| Check | Result |
| --- | --- |
| Size | 14.9 GB on disk, 4.40 GB compressed; about 12 to 15 minutes to build on the reference laptop |
| `torch_probe.py` | PASS: a convolution on the GPU, capability (12, 0); exits 1 with CUDA hidden |
| Whole workspace in the image, no symlink install | 22 of 22, `rosdep check` exit 0 |
| Python node modules | 36 of 36 import |
| As the simulator container | smoke 18 of 18 and autonomy PASS (2026-09-29); the final-tree runs are in report section 20 |
| YOLOPv2 against the simulator, `lane_eval_sim.py`, 100 s, 640x360 | **hit rate 0.166** at 0.25 m against a ground-truth ceiling of 0.676; runs, not yet good enough to steer by |
| P0-1, a node subscribing to nothing beside the simulator, 60 s | **0** "sequence size exceeds remaining buffer" lines from this image, **3** from the retired Humble image, which also discovered **no** node names from the Jazzy graph |
| The committed Dockerfile reproduces it | a rebuild hit the cache on every step and kept the original creation time; its id differs only by the labels compose adds |

**Why P0-1 happened, mostly settled:** `rmw_dds_common`'s `Gid` is
`char[24]` on Humble and `char[16]` on Jazzy (`RMW_GID_STORAGE_SIZE`, read
from both images). Node names travel in `/ros_discovery_info` messages built
from Gids, so a Humble reader misparses Jazzy's, prints the line and learns no
node names, while topic data still flows through DDS endpoint discovery. The
sizes and the missing names are measured; the link is inferred. Practical
consequence while the robot side is still Humble: **`ros2 node list` from a
Humble container shows an empty Jazzy graph**. The reverse, a Jazzy tool
looking at Humble nodes, was not tested.

Three things the Humble recipe did that break on noble, each MEASURED:
`pip3 install -U pip` refuses (the system Python is "externally managed",
PEP 668); `setuptools==58.2.0` cannot be imported on Python 3.12, so colcon
fails; and uninstalling `opencv-python` also deletes the headless build's
`cv2`, which is why **the Humble image has been running apt's OpenCV 4.5.4
all along**. And one more found while building this one: pip "upgrades" a
package apt installed by deleting apt's files (it did so to `setuptools`,
`sympy` and `mpmath` on the first build), so those go in first with
`--ignore-installed`.

---

## 8. How this was measured, and how to check it again

| Claim | Command or script | Where the output is |
| --- | --- | --- |
| every package is in section 2 | `python3 scripts/check_jazzy_inventory.py` | prints the counts; exit 0 |
| a clean install brings in what the packages use | clean `ros:jazzy-ros-base`, repo mounted read-only: `rosdep install --from-paths src --ignore-src -y` (no `-r`), then `rosdep check`, then `colcon build` without `--symlink-install`, `ros2 pkg executables`, and an import of every Python module | report section 20 |
| torch works on the GPU | `scripts/gazebo/measurements/torch_probe.py` in `igvc_dev_jazzy`; exit 0 only if a convolution runs on the GPU and the GPU's architecture is compiled in | report section 20 |
| the DDS defect is gone | `scripts/gazebo/measurements/dds_idle_probe.py`, a node that subscribes to nothing, beside the running simulator | report section 20 |
| YOLOPv2 lane quality | `scripts/gazebo/lane_eval_sim.py` on YOLOPv2's remapped `/lane_map`, Hough and the ground truth scored in the same drive | report section 20 |

The 2026-09-24 inventory that this page is built on (per-package build logs,
the ros2_control runtime probe, the apt and pip checks, the competition-branch
dry-run merge) was a local working document; its findings are carried here
with their evidence, and each web claim was re-checked against its source by
a second agent before being used.

## 9. Corrections to earlier statements

| Earlier statement | Where | Now |
| --- | --- | --- |
| "Do all 22 packages build on Jazzy?" is open | `DOCKER_CHANGES.md` section 8 item 13 | **Yes**, measured three times |
| "Adopting the competition branch is a port, not a merge" | earlier notes, `BRANCHES.md` | true only for a wholesale checkout; a merge has 1 trivial conflict and two fixups (section 4) |
| "Which lane detector competed is unknown" | earlier notes | the branch tip runs the rewritten `lane_segmentation`, YOLOPv2 TorchScript, front camera |
| The real robot's drive needs `twist_stamper` "on Jazzy" | the 2026-09-24 plan | it needed it on Humble too (6.1); the gap was packaging |
| Switch the lidar to the `rplidar_ros` binary | the 2026-09-24 inventory | no: that release predates C1 support (2.1) |
| torch has no rosdep key on noble | the 2026-09-24 inventory | the key exists; the apt package it names does not (6.3) |
| `igvc-dev-jazzy` would be about 11.3 GB | the 2026-09-24 plan (inferred) | **14.9 GB** on disk (`docker image ls`), 4.40 GB compressed (`docker image inspect`); 5.7 GB of it is pip content (NVIDIA CUDA libraries 3.12 GB, torch 1.15, triton 0.88) |
| The 2026 cameras ran in the Humble `jetson-zed` container | an audit verifier, 2026-09-24 | on the competition branch they ran in `dazzling_easley`, under Jazzy (section 5) |
| P0-1 is "a discovery-phase parse failure, hypothesis" | `GAZEBO_TODO.md`, 2026-09-17 | gone with a Jazzy perception container; a Humble reader also discovers **no node names** from Jazzy, consistent with `rmw_dds_common`'s Gid shrinking from 24 to 16 bytes (section 7) |
| The 2027 DevEnv uses `osrf/ros:jazzy-desktop` | session notes | DevEnv `main` uses `ghcr.io/gold-rush-robotics/dev_env:9`, built `FROM ros:jazzy-ros-base`; the `jazzy-desktop` PR was closed unmerged |
