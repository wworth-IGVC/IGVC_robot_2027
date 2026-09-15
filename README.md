# IGVC_robot_2027

ROS 2 workspace for Gold Rush Robotics' 2027 IGVC robot, forked from [Gold-Rush-Robotics/IGVC_Robot_2026](https://github.com/Gold-Rush-Robotics/IGVC_Robot_2026). The repo contains robot description files, ros2_control hardware interfaces, bringup launch files, ZED/LiDAR/GPS integration, lane perception and navigation nodes, Isaac/Genesis simulation support, Docker runtime environments, and model training assets.

## What changed in this fork

- **The Humble Docker image now builds.** It previously failed on every one of
  the 22 workspace packages; it now builds all 22 clean. The root cause was a
  `setuptools` version conflict that aborted the run before the other 21
  packages were even attempted.
- **The private ZED image has an open replacement.**
  `docker/Dockerfile.igvc-zed-humble` builds an equivalent x86 image from public
  sources, so you no longer need pull access to
  `ghcr.io/gold-rush-robotics/dev-zed`.
- **Windows is supported.** `scripts/setup-windows.ps1` checks and installs the
  prerequisites, and `docker-compose.windows.yml` works on Docker Desktop.

- **The simulator works, and the robot drives the IGVC course by itself.**
  Gazebo Harmonic on ROS 2 Jazzy, running the real 2026 navigation stack -
  ground-truth lane grid, IGVC navigator, Nav2 - with none of those nodes
  modified. It is **not** perceiving anything yet; the lane lines and barrels
  come from `track_points.json`. See
  [docs/GAZEBO_SETUP.md](docs/GAZEBO_SETUP.md) sections 9 and 10.

Every change, its root cause, and how it was verified is recorded in
[docs/DOCKER_CHANGES.md](docs/DOCKER_CHANGES.md).

## Start here

| You want to | Read |
| --- | --- |
| **Run the simulator and watch the robot drive the course** | [docs/IGVC_2027_Gazebo_Setup_Guide.docx](docs/IGVC_2027_Gazebo_Setup_Guide.docx), or [docs/GAZEBO_QUICKSTART.md](docs/GAZEBO_QUICKSTART.md) |
| Set up a machine from scratch, new to Docker | [docs/IGVC_2027_Docker_Setup_Guide.docx](docs/IGVC_2027_Docker_Setup_Guide.docx) |
| Understand how the simulator works, or change it | [docs/GAZEBO_SETUP.md](docs/GAZEBO_SETUP.md) |
| **See what is done and what is left on the simulator** | [docs/GAZEBO_TODO.md](docs/GAZEBO_TODO.md) |
| Know what is in each image and why | [docs/DOCKER_CHANGES.md](docs/DOCKER_CHANGES.md) |
| Find the right branch | [docs/BRANCHES.md](docs/BRANCHES.md) |

The Gazebo quickstart has a section for machines unlike the one it was built
on - Windows 10, native Linux, AMD/Intel GPUs, and no GPU at all - because most
"it works for you and not for me" reports trace back to one of those.

## Repository layout

| Path | Purpose |
| --- | --- |
| `src/` | ROS 2 packages for bringup, robot description, hardware, perception, simulation, and vendored drivers/interfaces. |
| `docker/` | Docker image definitions: the Humble fused-drive runtime image and the consolidated ZED + Humble image. |
| `docker-compose.yml` | GPU-enabled compose services for the Jazzy ZED/dev environment, the Humble fused-drive runtime, and the consolidated ZED image. |
| `docker-compose.windows.yml` | Same services adapted for Docker Desktop on Windows, which cannot use `/dev`, host networking, or `runtime: nvidia`. |
| `scripts/` | Workspace helper scripts for DDS setup, rosbag/data export, and Windows prerequisite setup (`setup-windows.ps1`). |
| `training/` | YOLOv12 training workflow, dataset, training requirements, seed weights, and training outputs. |
| `IGVC_track_generator/` | Procedural IGVC-style course/track generation utilities. |
| `isaac/` | Isaac Sim project assets and extensions. |
| `isaacsim/` | Local Isaac Sim installation/support files. This is large and usually machine-specific. |
| `odrive_config/` | ODrive controller configuration snapshots. |
| `models/` | Runtime model weights such as `yolopv2.pt`; these are ignored by Git. |
| `build/`, `install/`, `log/` | Local colcon outputs. These are generated and ignored. |
| `yolopv2_bag/` | Local bag/data workspace for YOLOPv2 experimentation. |

## Runtime targets

The workspace is consolidating on **ROS 2 Jazzy**. Humble reaches end of life in
May 2027 and the competition is 4 to 8 June 2027, so Humble would be
unsupported before the team ever competes on it. Jazzy runs to May 2029.

| Target | Use case | Status |
| --- | --- | --- |
| ROS 2 Jazzy in Docker | **The simulator.** Gazebo Harmonic, `ros_gz`, `gz_ros2_control`. | **Supported, and the robot drives the course autonomously.** `docker/Dockerfile.gazebo-jazzy`. ~5.6 GB on disk, now including Nav2. |
| ROS 2 Humble in Docker | Fused-drive runtime, still the default for everyday work. | **Pending port to Jazzy.** `docker/Dockerfile.humble-fused-drive`. |
| ROS 2 Humble + ZED in Docker | ZED camera work and robot machines. | **Pending port to Jazzy.** `docker/Dockerfile.igvc-zed-humble` — CUDA 13, ZED SDK, ZED ROS 2 wrapper. 28.6 GB on disk. |
| ROS 2 Jazzy on host | Local debug GUI and host-side tools. | Host tools can see Docker topics when the shared DDS helper is sourced. |

The two Humble images still work and have not been moved; they are the everyday
path until their Jazzy replacements are built and verified. Retired images live
in [docker/deprecated/](docker/deprecated/) with a README explaining what
replaced them.

Jazzy is where the **code** already was, though not the robot images. The 2026
competition branch is Jazzy code, the competition robot ran out of a
`jazzy_ws`, the org's `dev_env` image is Jazzy, and the 2027 team standardised
on `osrf/ros:jazzy-desktop`. The org's **robot-side** images — `jetson-zed`,
`jetson-ros-base`, `jetson-isaac-ros`, `isaac-ros` — are genuinely Humble; see
§5.4 of [docs/DOCKER_CHANGES.md](docs/DOCKER_CHANGES.md), and beware two
documented traps there: `docker-compose.jetson.yml`'s "ROS 2 Jazzy" comments
are stale, and `jetson-ros-base` is tagged `jazzy-36.4.7-2` while its
Dockerfile builds Humble.

So the robot side is a real migration, not a formality, and nothing here has
started it. The EOL date decides it anyway — Humble is unsupported before the
team competes on it.

`docker/Dockerfile.igvc-zed-humble` still does the job it was written for — it
replaces a **private** ZED image that most team accounts cannot pull and that
has no published build recipe. The image it replaces, `dev-zed`, is itself
Jazzy, so porting this one to Jazzy moves it closer to what it stands in for.
See [docs/DOCKER_CHANGES.md](docs/DOCKER_CHANGES.md).

## ROS packages

### Project packages

| Package | Role |
| --- | --- |
| `igvc_test_bringup` | Launch/config package for motor controllers, fused drive, lane segmentation, ZED odometry, Nav2, RViz, teleop, and simulation bringup. |
| `igvc_test_description` | Robot URDF/Xacro, meshes, sensor mounts, ros2_control tags, and generated URDFs. |
| `grr_hardware` | ros2_control hardware plugins for robot drive interfaces. Exports `grr_hardware/CanInterface` and `grr_hardware/IsaacDriveHardware`. |
| `igvc_lane_detection` | Python perception/navigation stack: lane detection, YOLOPv2 lane segmentation, localization, navigator, odom/TF bridge, and projection utilities. |
| `igvc_simulation_interface` | Python client wrapper around simulation interfaces for loading worlds, spawning entities, stepping/resetting simulation, and Isaac/Genesis integration. |
| `igvc_lidar_test` | LiDAR test package that depends on `sllidar_ros2`. |
| `debug_gui` | PyQt debug GUI for direct robot commands and camera/image feedback. |

### Vendored or third-party packages

| Package group | Role |
| --- | --- |
| `sllidar_ros2` | Slamtec/RPLiDAR ROS 2 driver and view launch files. |
| `zed_description` | Stereolabs ZED camera description package. |
| `ros_odrive/` | ODrive CAN and ros2_control packages plus a botwheel explorer demo. |
| `ros2-ublox-zedf9p/` | u-blox ZED-F9P GPS driver, messages, serialization, and metapackage. |
| `genesis_ros/` | Genesis simulation packages, simulation interfaces, and ros2_control demos. |

## Key launch files

Launch files live mainly in `src/igvc_test_bringup/launch/`.

| Launch file | Purpose |
| --- | --- |
| `igvc_fused_drive.launch.py` | Main fused-drive stack. Brings up motor controllers, lane segmentation/Nav2, odom-to-TF, and twist stamping. |
| `motor_controllers.launch.py` | Robot description, `ros2_control_node`, joint state broadcaster, and diff drive controller. |
| `teleop.launch.py` | Includes motor controllers and adds joystick teleop via `joy` and `teleop_twist_joy`. |
| `lane_segmentation.launch.py` | YOLOPv2 lane segmentation, localization, navigator, and Nav2. |
| `lane_follower.launch.py` | Classic lane follower path using the non-deep-learning lane detector. |
| `navigation_no_docking.launch.py` | Nav2 bringup without docking behavior. |
| `zed_multi.launch.py` | Multi-ZED camera bringup. |
| `zed_multi_fused_odom.launch.py` | ZED odometry plus robot localization fusion. |
| `simulation_launch.launch.yaml` | Simulation bringup used by the ZED/dev compose service. |
| `simulation_interface.launch.yaml` | Simulation interface launch entry. |
| `rviz.launch.py` | RViz startup using the project config. |

Common commands:

```bash
ros2 launch igvc_test_bringup motor_controllers.launch.py hardware_interface:=CanInterface
ros2 launch igvc_test_bringup teleop.launch.py hardware_interface:=CanInterface
ros2 launch igvc_test_bringup lane_segmentation.launch.py model_weights:=$PWD/models/yolopv2.pt
ros2 launch igvc_test_bringup igvc_fused_drive.launch.py hardware_interface:=IsaacDriveHardware use_sim_time:=true
```

## Build and local workspace use

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
```

For a narrower runtime build matching the Humble container package set:

```bash
colcon build --symlink-install \
	--base-paths src \
	--packages-select \
		zed_description \
		igvc_test_description \
		grr_hardware \
		igvc_lane_detection \
		igvc_test_bringup
```

Useful local tools:

```bash
ros2 run debug_gui debugger
ros2 run igvc_lane_detection lane_segmentation_node
ros2 run igvc_lane_detection navigation_node
ros2 run igvc_simulation_interface simulation_interface
```

## Humble Docker bringup

The repo includes a ROS 2 Humble container for running the fused-drive stack on systems where the host ROS install is different, such as Jazzy.

Build the dependency image once:

```bash
docker compose build igvc_humble_fused_drive
```

Launch the Humble fused-drive stack:

```bash
docker compose up igvc_humble_fused_drive
```

The container runs:

```bash
ros2 launch igvc_test_bringup igvc_fused_drive.launch.py hardware_interface:=IsaacDriveHardware use_sim_time:=true
```

The Humble image definition lives in `docker/Dockerfile.humble-fused-drive`. The compose service keeps `build`, `install`, and `log` in Docker volumes so restarts can reuse colcon output instead of rebuilding from scratch each time.

To force a clean Humble workspace rebuild:

```bash
docker compose down -v
docker compose up igvc_humble_fused_drive
```

`down -v` removes the named volumes declared in `docker-compose.yml`, so it works
regardless of what the checkout directory is called. Removing them by hand does
not: Compose prefixes volume names with the directory, so in a fork checked out
as `IGVC_robot_2027` they are `igvc_robot_2027_igvc_humble_*`, not
`igvc_robot_2026_*`.

Run `down -v` after rebuilding an image, too. Named volumes are seeded from the
image only when they are first created, so a stale volume will quietly keep
serving the old build output.

## Windows / Docker Desktop

There are two ways to run this on Windows. If you have WSL2 with a Linux
distribution installed, prefer the first - it is closer to how the robot
actually runs and it gives you working GUI tools.

### Option A - from inside WSL2 (recommended)

Open your WSL distribution (`wsl -d <YourDistro>`), `cd` to the repo, and use
the normal Linux compose file:

```bash
docker compose build igvc_humble_fused_drive
docker compose run --rm igvc_humble_fused_drive
```

Inside WSL2, `/dev`, `/tmp`, and `/tmp/.X11-unix` are real Linux paths and
`network_mode: host` applies to the WSL2 VM, so `docker-compose.yml` works
essentially as written. On Windows 11, WSLg supplies an X server, which means
`rviz2` and other GUI tools actually display - they cannot from PowerShell.

Docker Desktop shares its daemon with WSL when the distro is enabled under
Settings > Resources > WSL Integration, so you do not install Docker twice.

**Gazebo must be run this way, and it needs the Windows compose file even from
WSL2**, because the `igvc_gazebo` service is defined there:

```bash
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
docker compose -f docker-compose.windows.yml up -d igvc_gazebo

# prove the GPU is actually in use before anything else
docker exec -it igvc_gazebo bash -c   "source /opt/ros/jazzy/setup.bash && bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"

# the robot drives the course by itself
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

**`up -d`, not `run --rm`.** `run` creates a new container every time, so a
second `run` gives you a second *simulator* rather than a second shell into the
first. Two of them on one ROS domain both publish `/clock`, `/odom` and `/tf`
for the same robot, and the result looks like a navigation bug. On native Linux
use `docker compose up -d igvc_gazebo_linux` instead - same image, different
GPU and display plumbing.

Verified 2026-09-15: GUI **and** GPU, `OpenGL renderer string: D3D12 (NVIDIA
GeForce RTX 5070 Ti Laptop GPU)`. From PowerShell the same service still runs
headless but can never show a window, because Docker Desktop's own VM has no
display of any kind. The distro version does not matter - ROS lives in the
container - so Ubuntu 26.04 is a perfectly good launcher. See
[docs/GAZEBO_SETUP.md](docs/GAZEBO_SETUP.md) section 3A.

### Option B - from PowerShell

If you do not have WSL2 set up, use the Windows-specific compose file, passed
explicitly with `-f`:

```powershell
docker compose -f docker-compose.windows.yml build igvc_humble_fused_drive
docker compose -f docker-compose.windows.yml run --rm igvc_humble_fused_drive
```

Name the service. A bare `docker compose build` builds **every** service in the
file, which now includes both ZED variants — roughly 40 minutes and 70 GB when
all you wanted was the everyday image.

This drops the host-Linux-only settings (`/dev`, `/tmp/.X11-unix`,
`network_mode: host`, `runtime: nvidia`) that Docker Desktop cannot honour, and
keeps GPU access through `gpus: all`. GUI tools will not display in this mode.

### Automated setup

`scripts/setup-windows.ps1` checks every prerequisite - Windows build, WSL2,
Docker, GPU passthrough, disk space, submodules - then downloads the YOLOPv2
weights and builds the image:

```powershell
.\scripts\setup-windows.ps1            # check, then build
.\scripts\setup-windows.ps1 -CheckOnly # diagnose only, change nothing
.\scripts\setup-windows.ps1 -WithZed   # also build the large ZED image
```

Windows 10 users need 22H2 (build 19045) or newer; current Docker Desktop
refuses to install on earlier builds. The script reports this.

### GPU notes

RTX 50-series (Blackwell, compute capability 12.0) requires CUDA 12.8 or newer.
The images here use CUDA 13, and the bundled PyTorch (2.14 + cu130) ships
`sm_120` kernels, so 50-series laptops work without recompiling anything.

Install the NVIDIA driver on **Windows** only. Do not install Linux NVIDIA
drivers inside WSL - that breaks the passthrough.

**CUDA passthrough is not the same as GPU rendering.** `--gpus all` gets you
CUDA, and that is all it gets you. There is no native NVIDIA OpenGL driver in a
WSL2 container: GL goes through Mesa's `d3d12` driver on top of `/dev/dxg`, and
if that driver cannot load, Mesa falls back to `llvmpipe` software rendering
**silently**, with no error and no warning. Gazebo will start, load a world and
publish camera frames while rendering entirely on the CPU.

The `igvc_gazebo` service already carries the four settings that fix this, and
`scripts/gazebo/render_check.sh` is the one-command check. Anything else doing
GPU rendering in a container needs the same treatment - see
[docs/GAZEBO_SETUP.md](docs/GAZEBO_SETUP.md) section 3. On a dual-GPU laptop
(Intel iGPU plus NVIDIA) you must also set
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, or Mesa picks the Intel adapter and
the process aborts inside Intel's WSL driver.

### ZED cameras on Windows

A ZED camera cannot be reached from Docker Desktop, and `/dev` inside WSL2 has
no USB devices by default, so the camera nodes will not run on a Windows laptop.
`usbipd-win` can attach USB devices into WSL2 and is worth trying, but ZED
cameras are high-bandwidth USB3 and USB/IP handles that poorly - do not plan
around it without testing. Use the Jetson or a native Linux machine for camera
work.

Note: `docker compose` derives volume names from the checkout directory, so in
this fork they are prefixed `igvc_robot_2027_`. Use `docker compose down -v`
rather than naming volumes by hand.

## Docker services

| Service | Image | Purpose |
| --- | --- | --- |
| `igvc_dev_zed` | `ghcr.io/gold-rush-robotics/dev-zed:5.1.0-13.0.0` | Jazzy/ZED development container. Builds `igvc_test_bringup` and launches `simulation_launch.launch.yaml`. |
| `igvc_humble_fused_drive` | Built from `docker/Dockerfile.humble-fused-drive` | Humble runtime container for `igvc_fused_drive.launch.py` with GPU, host networking, DDS profile, and persistent colcon volumes. **Everyday default** — 3.98 GB. |
| `igvc_zed_humble` | Built from `docker/Dockerfile.igvc-zed-humble` | Everything the fused-drive image has, plus the ZED SDK and ZED ROS 2 wrapper. Use for ZED camera work and on robot machines. 28.6 GB on disk. |
| `igvc_zed_humble_upstream` | Same Dockerfile, different build args | Same as above but built against **upstream** `zed-ros2-wrapper` v5.4.1 with SDK 5.3.0 — the same wrapper and SDK as the Jetson image. See below. |
| `igvc_gazebo` | Built from `docker/Dockerfile.gazebo-jazzy` | **Gazebo Harmonic** (`gz-sim` 8.15.0) + **Jazzy** + `ros_gz` + `gz_ros2_control`. The simulator for everyday development, plus Nav2. ~5.6 GB on disk. Run it from WSL2; `igvc_gazebo_linux` in `docker-compose.yml` is the native-Linux twin. See [docs/GAZEBO_SETUP.md](docs/GAZEBO_SETUP.md). |

All three services mount the repository at `/root/ros2_ws/src/IGVC_robot_2026`, use host networking, expose `/dev`, share `/tmp/.X11-unix`, and request NVIDIA GPU access.

### Which one should I use?

Use **`igvc_humble_fused_drive`** unless you are working on ZED camera code. It
is a quarter the size, and all 22 workspace packages build in it without the ZED
SDK.

`igvc_zed_humble` exists to replace `igvc_dev_zed`, which pulls a **private**
image (`ghcr.io/gold-rush-robotics/dev-zed`) that most team accounts cannot
access and for which no build recipe is published anywhere. The new image is
built from a Dockerfile in this repo, so anyone can build it:

```bash
docker compose build igvc_zed_humble     # ~18 minutes, 28.6 GB on disk
```

On Windows, note that Docker Desktop cannot pass USB devices through, so a ZED
camera is not usable from a Windows container regardless of image. See
[docs/DOCKER_CHANGES.md](docs/DOCKER_CHANGES.md) for the full rationale.

### Two ZED variants — pick one

The ZED wrapper only accepts a **specific range** of ZED SDK versions, and it
enforces this **when the node starts**, not when the image builds. An image can
build perfectly, install every ZED package, and then die the instant you launch
it:

```text
[ERROR] This version of the ZED ROS2 wrapper is designed to work with
        ZED SDK v4.2 or newer up to v5.2.
[INFO]  * Detected SDK v5.3.0 ... Node stopped.
```

So the wrapper and the SDK have to be chosen together. Two combinations are
verified, and both are wired up:

| | `igvc_zed_humble` | `igvc_zed_humble_upstream` |
| --- | --- | --- |
| Wrapper | org fork v5.2.1 | upstream v5.4.1 (pinned) |
| ZED SDK | 5.2.3 | 5.3.0 |
| Matches `jetson-zed` | no | **yes** |
| Wrapper currency | 70 commits behind upstream | current |

```bash
docker compose build igvc_zed_humble            # default
docker compose build igvc_zed_humble_upstream   # matches the Jetson
```

They use separate colcon volumes, so both can coexist. Which one the team
standardises on is still an open decision — see
[docs/DOCKER_CHANGES.md](docs/DOCKER_CHANGES.md) §4.5 for the evidence behind
each.

**If you change the SDK or wrapper version, test by launching the node, not by
building the image.** `DOCKER_CHANGES.md` §7.2 has the exact command and how to
read its output.

### Disk space

Docker reports two different sizes and they differ by about 3.3×:

| | Download | On disk |
| --- | --- | --- |
| `igvc_humble_fused_drive` | 3.98 GB | 13 GB |
| `igvc_zed_humble` | 9.70 GB | 28.6 GB |

`docker image inspect` reports the compressed download size. `docker image ls`
reports the unpacked footprint — but layers are unpacked **lazily, on the first
container run**, so a freshly built image looks smaller than it will be:

```text
igvc-zed-humble  upstream  9.7GB     # right after building it
igvc-zed-humble  upstream  28.6GB    # after running it once
```

**Plan around the on-disk column**, and leave room for build cache — it reached
55.9 GB while these images were being developed.

```bash
docker system df        # see images, volumes, and build cache
docker builder prune    # reclaim build cache
```

## Shared DDS profile

Humble-in-Docker and local Jazzy tools must use the same DDS settings to see the same ROS graph. Both compose services use the shared Fast DDS profile at:

```text
src/igvc_test_bringup/config/fastdds_udp.xml
```

Before running local Jazzy commands that need to see Docker topics, source:

```bash
source scripts/use_igvc_dds.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
```

The helper sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, and the shared Fast DDS profile path. If you change `ROS_DOMAIN_ID`, use the same value for both the local shell and Docker compose.

## Perception and navigation

The main perception/navigation stack is in `igvc_lane_detection` and is launched through `igvc_test_bringup`.

| Node | Executable | Purpose |
| --- | --- | --- |
| Lane detector | `lane_detection_node` | Classic image/depth lane detection and occupancy grid generation. |
| YOLOPv2 segmenter | `lane_segmentation_node` | Deep segmentation pipeline using `yolopv2.pt`, depth projection, occupancy grid output, overlays, and markers. |
| Localization | `localization_node` | Localization state handling for sim, dead reckoning, and GPS-enabled modes. |
| Navigator | `navigation_node` | Converts lane/costmap information into Nav2 waypoints/goals. |
| Odom/TF bridge | `odom_tf_bridge_node` | Publishes transform data from odometry when needed. |
| Multi-camera detector | `multi_camera_lane_detection_node` | Multi-camera lane fusion path. |

Runtime model weights are expected at `models/yolopv2.pt` or can be passed through `model_weights:=...`. The Humble Docker service sets:

```text
YOLOPV2_WEIGHTS=/root/ros2_ws/src/IGVC_robot_2026/models/yolopv2.pt
```

To fetch or prepare YOLOPv2 runtime weights, see:

```bash
src/igvc_lane_detection/scripts/fetch_yolopv2_weights.sh
```

## Robot description and control

Robot description assets live in `src/igvc_test_description/`.

| Path | Purpose |
| --- | --- |
| `urdf/robots/test_robot.urdf.xacro` | Main robot Xacro entry point. |
| `urdf/control/ros2_control_info.urdf.xacro` | ros2_control hardware interface wiring. |
| `urdf/parts/` | Body, ZED camera, GPS antenna, and RPLiDAR Xacro parts. |
| `meshes/` | Robot and sensor meshes. |
| `igvc_robot.urdf`, `test_robot.urdf` | Generated or exported URDF artifacts. |

`grr_hardware` exports ros2_control system plugins:

| Plugin | Use case |
| --- | --- |
| `grr_hardware/CanInterface` | Physical robot CAN/motor-controller interface. |
| `grr_hardware/IsaacDriveHardware` | Isaac/simulation-facing drive interface. |

Motor controller parameters are in `src/igvc_test_bringup/config/controllers.yaml`.

## Sensors and configuration

Important bringup configs live in `src/igvc_test_bringup/config/`.

| Config | Purpose |
| --- | --- |
| `controllers.yaml` | ros2_control controller configuration. |
| `xbox-holonomic.config.yaml` | Joystick teleop mapping. |
| `lane_segmentation_config.yaml` | YOLOPv2 lane segmentation parameters. |
| `lane_detection_config.yaml` | Classic lane detector parameters. |
| `multi_camera_lane_detection.yaml` | Multi-camera lane detection parameters. |
| `navigator_config.yaml` | Lane navigator and waypoint behavior. |
| `nav2_lane_follow_config.yaml` | Nav2 parameters for lane-following behavior. |
| `igvc_nav_to_pose_bt.xml` | Nav2 behavior tree XML. |
| `zed_multi_ekf.yaml` | Robot localization EKF for multi-ZED odometry. |
| `zed_f9p.yaml` | u-blox ZED-F9P GPS parameters. |
| `common_stereo.yaml` | Shared ZED stereo camera configuration copied into the ZED wrapper container. |
| `twist_mux.yaml` | Twist multiplexer configuration. |
| `config.rviz` | RViz visualization config. |
| `fastdds_udp.xml` | Shared Fast DDS UDP profile. |

## Training assets

YOLOv12 training files live under `training/` so the ROS workspace root stays focused on bringup and runtime code.

```bash
cd training
pip install -r training_requirements.txt
./train_yolov12.sh --help
```

See `training/README.md` for the full training workflow. Training data is in `training/dataset/`, seed weights are in `training/weights/`, and outputs are written under `training/runs/`.

Training layout:

| Path | Purpose |
| --- | --- |
| `training/train_yolov12.py` | Python training entry point using Ultralytics. |
| `training/train_yolov12.sh` | Shell wrapper with common defaults. |
| `training/training_requirements.txt` | Python training dependencies. |
| `training/dataset/` | Roboflow/YOLO dataset. |
| `training/weights/` | Seed model weights such as `yolov12n.pt`. |
| `training/runs/` | Training outputs. |

## Simulation and course generation

Simulation support is split across several directories:

| Path | Purpose |
| --- | --- |
| `src/igvc_simulation_interface/` | ROS 2 Python interface for simulation services. |
| `src/genesis_ros/` | Genesis simulator bridge packages and interface definitions. |
| `isaac/` | Isaac-specific assets/extensions. |
| `isaacsim/` | Local Isaac Sim installation tree and helper scripts. |
| `IGVC_track_generator/` | Procedural IGVC course generator; see `IGVC_track_generator/README.md`. |

The track generator can produce IGVC-style course imagery and related assets for simulation/training experiments.

## Data utilities

Workspace-level scripts in `scripts/`:

| Script | Purpose |
| --- | --- |
| `use_igvc_dds.sh` | Exports shared DDS settings for local shells. |
| `bag_to_csv_and_images.py` | Converts ROS bag data into CSV/images for inspection or dataset preparation. |
| `bag_to_yolopv2_overlay.py` | Generates YOLOPv2 overlay artifacts from bag data. |

## Generated and ignored content

These directories/files are treated as generated, local, or large artifacts:

| Pattern | Reason |
| --- | --- |
| `build/`, `install/`, `log/` | Colcon outputs. |
| `training/dataset/`, `training/runs/`, `training/weights/*.pt` | Training data, outputs, and local weights. |
| `models/`, `*.pt` | Runtime model weights. |
| `isaacsim/` | Local Isaac Sim install/support tree. |
| `rosbags/`, `rosbag_export_test/`, `yolopv2_bag/` | Local bag/export data. |
| `**.pyc`, `.venv/` | Python generated files and virtual environments. |

## Common workflows

### Run the Humble fused-drive stack

```bash
docker compose build igvc_humble_fused_drive
docker compose up igvc_humble_fused_drive
```

### Inspect Humble topics from a Jazzy shell

```bash
source scripts/use_igvc_dds.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list
```

### Run teleop with physical CAN hardware

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch igvc_test_bringup teleop.launch.py hardware_interface:=CanInterface
```

### Run teleop against the Isaac drive interface

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch igvc_test_bringup teleop.launch.py hardware_interface:=IsaacDriveHardware use_sim_time:=true
```

### Run the debug GUI

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run debug_gui debugger
```

### Validate compose and training script syntax

```bash
bash -n training/train_yolov12.sh
python3 -m py_compile training/train_yolov12.py
docker compose -f docker-compose.yml config >/tmp/igvc_compose_config.yml
```

## Notes for maintainers

- Keep ROS launch/config files inside `src/igvc_test_bringup/` unless they belong to a specific package.
- Keep Docker image definitions under `docker/` and wire them from `docker-compose.yml`. If you add a service there, add it to `docker-compose.windows.yml` as well, or Windows machines silently lose it.
- Pin `setuptools` after any change to the pip stack in a Dockerfile. `torch`/`ultralytics` pull setuptools past 80, which breaks `colcon-core` and every `ament_python` package. See `docs/DOCKER_CHANGES.md` §2.1.
- Keep training-only data and scripts under `training/`.
- Keep runtime model weights in `models/` or pass absolute paths with launch arguments.
- Use the shared DDS profile whenever local tools and Docker containers need to participate in the same ROS graph.
