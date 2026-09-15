# Branches in this fork

## Why there are 15 `2026/` branches

`Gold-Rush-Robotics/IGVC_robot_2026` has 15 branches and the competition work was
never merged into `main`. This fork originally descended from `main` alone, which
is a pre-competition snapshot from 2026-05-31.

Every upstream branch is mirrored here under a `2026/` prefix. They are **exact
mirrors**: nothing has been committed on top of them, so they can be diffed
against upstream at any time. The prefix keeps them clearly separate from active
2027 work and guarantees none can collide with `main`.

Mirroring cost nothing in disk terms; the objects were already present locally.

```bash
git fetch upstream --prune
for b in $(git for-each-ref --format='%(refname:strip=3)' refs/remotes/upstream/ | grep -v '^HEAD$'); do
    git branch --force "2026/$b" "upstream/$b"
done
git push origin 'refs/heads/2026/*:refs/heads/2026/*'
```

## Inventory

`isaac/exts` is pinned to three different commits across these branches, which is
the single most useful thing to know before checking one out.

| Branch | `isaac/exts` | ahead of `2026/main` | last commit |
| --- | --- | --- | --- |
| `2026/more_diverging_changes` | 72ff9c21 | **61** | 2026-06-01 |
| `2026/gui_testing` | 3d856e5c | 40 | 2026-05-26 |
| `2026/test_cases` | 72ff9c21 | 34 | 2026-05-22 |
| `2026/tuning_parameters` | 72ff9c21 | 34 | 2026-05-20 |
| `2026/real_robot_nav2` | 72ff9c21 | 30 | 2026-04-20 |
| `2026/test_comp_changes_isaac_smi` | 72ff9c21 | 15 | 2026-05-29 |
| `2026/yolo_ros` | 3d856e5c | 14 | 2026-05-22 |
| `2026/zed` | 72ff9c21 | 13 | 2026-05-12 |
| `2026/obk_and_no_mans_fixes` | 72ff9c21 | 10 | 2026-05-29 |
| `2026/outdoor_testing` | 72ff9c21 | 5 | 2026-05-25 |
| `2026/full_self_driving` | 375eddd1 | 3 | 2026-05-27 |
| `2026/yolo26` | 72ff9c21 | 1 | 2026-05-22 |
| `2026/lidar_testing` | 72ff9c21 | 1 | 2026-04-04 |
| `2026/main` | 375eddd1 | 0 | 2026-05-31 |
| `2026/vision_sense` | 375eddd1 | 0 | 2026-05-24 |

`2026/more_diverging_changes` is 61 ahead of `main` and 0 behind, ending
"final push +1" on 2026-06-01, and is a strict superset of
`test_comp_changes_isaac_smi`. It was the leading candidate for what ran at the
2026 competition. **See "What actually ran at competition" below: the team lead
answered part of that on 2026-09-15, and the answer was `main`.**

## The `isaac/exts` pin, and why it is fragile

`.gitmodules` points `isaac/exts` at `https://github.com/stereolabs/zed-isaac-sim`
on every branch. Ten branches pin it to `72ff9c21`, a commit that exists only in
`Gold-Rush-Robotics/zed-isaac-sim`, the organisation's fork.

**This resolves today.** Verified from a clean clone of
`2026/more_diverging_changes` with no config overrides:
`git submodule update --init --recursive` exits 0 and lands `isaac/exts` at
`72ff9c21`. GitHub shares object storage across a fork network, so a fetch of a
specific SHA succeeds even from the parent repository's URL.

It is worth knowing what that depends on:

- **It breaks under `--depth 1`.** Shallow fetch of an arbitrary SHA is refused.
  Any CI job or clone script using shallow checkout will fail on these branches.
- **It breaks if the organisation's fork is deleted.** The commit exists in no
  other repository. Nothing in this fork preserves it.
- **It does not survive mirroring off GitHub.** Push these branches to GitLab, a
  self-hosted server, or a `git bundle`, and the submodule can no longer resolve.

So the setup is not broken, but it rests on a fork this team does not control.

## Submodule pointers do not follow a checkout

Pointers differ between branches and git will not update them when you switch.
Set this once:

```bash
git config submodule.recurse true
```

Without it, `git checkout 2026/more_diverging_changes` leaves `isaac/exts` at
whatever `main` pinned, and the ZED extension silently loads the wrong code.

## The permanent fix, when a baseline is chosen

Do not repoint `isaac/exts` at the organisation's fork. It is 2 ahead but **8
behind** upstream, so adopting it trades eight upstream commits for two, and one
of those two lowers the OmniGraph `TARGET_VERSION` from (2, 184, 5) to
(2, 184, 2), pinning to an older Isaac Sim.

The change actually needed is roughly 25 lines in
`exts/sl.sensor.camera/sl/sensor/camera/annotators.py`, which extracts a
`left_`/`front_`/`right_` prefix from the prim name so the extension can find
three cameras in a URDF-imported hierarchy instead of the hardcoded
`/base_link/<model>/CameraLeft`.

Fork `stereolabs/zed-isaac-sim` under an account this team controls, apply that
one file on top of the current pin, and point `isaac/exts` there. Upstream
history is kept, every branch resolves from a single URL this team owns, and the
dependency on the organisation's fork continuing to exist goes away.


---

## What actually ran at competition

**Answered 2026-09-15.** The team lead says **the Dockerfile on `main` of
`IGVC_robot_2026`** is what ran. Traced through both compose files on
`upstream/main` and verified against the `Gold-Rush-Robotics/docker_images`
repository, exactly **two** services are uncommented:

| Compose file | Active service | Image | Distro |
| --- | --- | --- | --- |
| `docker-compose.yml` | `igvc_dev_zed` | `dev-zed:5.1.0-13.0.0` | **Jazzy** |
| `docker-compose.jetson.yml` | `igvc_jetson_zed` | `jetson-zed:5.3-36.4.7` | **Humble** |

`igvc_humble_fused_drive` and `igvc_jetson_stack`
(`jetson-ros-base:humble-36.4.7`) are **both commented out**, on `main`, in
upstream. Both files use `network_mode: host` with a shared FastDDS/Zenoh
profile, so this was **one ROS graph spread across two machines**.

`jetson-zed` is Humble beyond doubt:
`docker_images/jetson-zed/Dockerfile` line 14 reads `ARG ROS2_DIST=humble`, on
base image `nvcr.io/nvidia/l4t-jetpack:r36.4.0`. Its `build` script sets
`VERSION="5.3-36.4.7"`, matching the tag in the compose file.

**Strong inference, which nobody has stated outright:** the drive and navigation
stack ran on a laptop rather than on the robot, because the Jetson's own stack
service is disabled. That reconciles two facts that otherwise contradict each
other - the competition code is Jazzy (`igvc_pointcloud_tools` fails to compile
on Humble, since `rclcpp::Clock::now()` is const in Jazzy and not in Humble)
while the Jetson container is Humble. They were different machines. **Worth
confirming with the team lead.**

### The part that is still open

`main` pins `isaac/exts` at `375eddd1`, stereolabs upstream. Per the section
above, upstream **cannot resolve three cameras** in a URDF-imported hierarchy;
that needs `72ff9c21`, which only the competition branches pin.

So a three-camera Isaac run from `main` should not have worked. **"Which
Dockerfile ran" and "which branch the ROS packages came from" may have different
answers**, and only the first has been answered. Ask the second separately.

---

## Org-wide branch audit, 2026-09-15

**The question:** does any other repository in the organisation hide competition
work, the way this one hides `more_diverging_changes`?

**No.** Every repository below was enumerated through the GitHub branches API,
not guessed.

| Repository | Default | Branches | Verdict |
| --- | --- | --- | --- |
| `IGVC_robot_2026` | `main` | **15** | the only one with competition branches |
| `docker_images` | `main` | 2 | `dev_env_packages` is dev tooling |
| `DevEnv` | `main` | 6 | nothing live outside `main`, see below |
| `hmi` | `main` | 8 | web feature and bugfix work |
| `discord-ros-tui` | `main` | 8 | UI work |
| `zed-ros2-wrapper-2025` | `master` | 1 | fork of `stereolabs/zed-ros2-wrapper` |
| `zed-isaac-sim` | `main` | 1 | fork of `stereolabs/zed-isaac-sim` |
| `yolo_ros` | `main` | 1 | fork of `mgonzs13/yolo_ros` |
| `VisionSense` | `main` | 1 | fork of `connected-wise/VisionSense` |
| `grr_hardware` | `main` | 1 | organisation original |
| `IGVC_track_generator` | `main` | 1 | fork of `lukelmg/procedural-tracks` |
| `flipsky_odesc_ros_odrive` | `main` | 1 | fork of `odriverobotics/ros_odrive` |

A keyword filter flagged `hmi/bugfix/component-updates`; that is a false
positive, because "component" contains "comp". Recorded so nobody
re-investigates it.

### `DevEnv`: only `main` is live

This matters because the 2027 team standardised DevEnv on
`osrf/ros:jazzy-desktop` with a `jazzy_ws`, and a branch literally named `jazzy`
invites the question of whether the real work lives there. **It does not.**

| Branch | Ahead | Behind | Files | Last commit |
| --- | --- | --- | --- | --- |
| `IGVC` | 0 | 0 | 0 | identical to `main` |
| `jazzy` | 0 | **4** | 0 | identical to `main`, just older |
| `windows` | 0 | **20** | 0 | identical to `main`, just older |
| `linux` | 4 | 11 | 91 | **2024-09-17** |
| `mac` | 6 | 11 | 76 | **2024-09-19** |

`IGVC`, `jazzy` and `windows` are stale pointers that `main` has overtaken -
there is nothing in them to merge. `linux` and `mac` are 2024-era leftovers
inherited from the `RoboEagles4828/DevEnv` upstream, not organisation work.
Working on `main` is correct and there is nothing to salvage elsewhere.

### What a clone without `--recursive` actually gets you

Worth stating because it has already bitten the team once. The nine submodules
are `grr_hardware`, `yolo_ros`, `ros_odrive`, `ros2-ublox-zedf9p`,
`sllidar_ros2`, `greenwave_monitor`, `zed-description`, `isaac/exts` and
`IGVC_track_generator`.

Clone without `--recursive` and `colcon build` succeeds on exactly seven
packages - `debug_gui`, `igvc_lane_detection`, `igvc_lidar_test`,
`igvc_simulation_interface`, `igvc_test_bringup`, `igvc_test_description`,
`ping_location` - because those are the only first-party packages in `src/`.

**It looks like a clean build and it is not a usable robot.** Missing: the
hardware interface, the entire perception stack, the camera URDF, the lidar
driver, motor control and GPS. Always:

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
git config submodule.recurse true
```
