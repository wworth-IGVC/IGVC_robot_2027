# Setting up the simulator: pick your machine

This page decides which setup path is yours. Everything after that choice
lives in one guide per platform under [`docs/setup/`](docs/setup/).

Everyone starts the same way, with a clone that includes the nine submodules.
Two of them are load-bearing: `zed_description` is built by name, and
`IGVC_track_generator` holds the course every navigation node reads.

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
```

## 1. Which path is yours

| Your machine | Path | Guide | After cloning, one command |
| --- | --- | --- | --- |
| **Windows 10 (22H2) or 11**, any GPU | Docker Desktop + WSL2 | [`docs/setup/WINDOWS.md`](docs/setup/WINDOWS.md) | `bash scripts/gazebo/bootstrap.sh` from a **WSL2** shell |
| **Mac with Apple Silicon** (M1 or newer, or the A18 MacBook) | pixi + RoboStack, native, **no Docker** | [`docs/setup/MACOS.md`](docs/setup/MACOS.md) | `pixi run bootstrap` |
| **Linux** x86_64 | pixi (native), or Docker | [`docs/setup/LINUX.md`](docs/setup/LINUX.md) | `pixi run bootstrap` |
| Intel Mac | **not supported** | | |

Both paths run the **same ROS 2 Jazzy + Gazebo Harmonic stack**, the same
launch files and the same check scripts in `scripts/gazebo/`. Only how the
software gets onto your machine differs.

## 2. What each path gives you, and how much of it is verified

"Verified" means a command ran and a number came back. Where a path has not
been run on its target machine, this says so.

| | Windows | Mac (native) | Linux (native) |
| --- | --- | --- | --- |
| Gazebo window | yes, via WSLg | yes, expected; see the guide | yes |
| GPU rendering | NVIDIA verified (tier A). Intel/AMD untested (tier B). No GPU works (tier C) | Metal, expected; **not yet run on a Mac** | your system's driver |
| Verified on | the reference RTX 5070 Ti laptop | the same environment on linux-64, in a clean container; **no Mac has run it yet** | a clean Ubuntu 24.04 container, software rendering |
| First install | ~10 min, 20 GB free disk | pixi install ~2 min + download, about 6 GB | same as Mac |

**If you own a Mac: you are the first.** Please work through
[`docs/setup/MACOS_CHECKLIST.md`](docs/setup/MACOS_CHECKLIST.md) and hand it
back. It is the only way this project gets real macOS numbers.

A Docker-based fallback for Macs exists (headless only, no window, software
rendering under emulation). Use it only if the native path fails; it is
described at the end of the Mac guide.

## 3. Daily use, side by side

| Task | Windows (in a WSL2 shell, repo root) | Mac / Linux (repo root) |
| --- | --- | --- |
| Start the container | `docker compose -f docker-compose.windows.yml up -d igvc_gazebo` | nothing to start |
| Start the simulator | `docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh` | `pixi run sim` |
| A second shell | `docker exec -it igvc_gazebo bash` then `source /root/ros2_ws/install/setup.bash` | `pixi shell` |
| Drive it by keyboard | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | same |
| Is the GPU rendering? | `docker exec igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh` | `pixi run render-check` |
| The topic contract, and does it drive | `... scripts/gazebo/bringup_smoke_test.sh` | `pixi run smoke` |
| Does it drive the course by itself | `... scripts/gazebo/autonomy_check.sh` | `pixi run autonomy` |
| Stop | `docker compose -f docker-compose.windows.yml stop` | Ctrl-C |

**Set a `ROS_DOMAIN_ID` of your own on every path**, before starting anything,
if anyone else on the same network is running ROS. Two simulators on one
domain both publish `/clock`, `/odom` and `/tf`, and the results look real and
are worthless. Windows: export it in the WSL2 shell before `compose up`. Mac
and Linux: `export ROS_DOMAIN_ID=<n>` before `pixi run`.

## 4. Which files belong to which path

| Path | Files |
| --- | --- |
| Windows | `scripts/setup-windows.ps1`, `docker-compose.windows.yml`, `scripts/gazebo/bootstrap.sh`, `docs/setup/WINDOWS.md`, `docs/setup/SESSION_COMMANDS.md` |
| Mac and Linux, native | `pixi.toml`, `pixi.lock`, `scripts/gazebo/bootstrap_native.sh`, `docs/setup/MACOS.md`, `docs/setup/MACOS_CHECKLIST.md`, `docs/setup/LINUX.md` |
| Mac fallback | `docker-compose.mac.yml` |
| Linux, Docker | `docker-compose.yml`, service `igvc_gazebo_linux` |
| **Shared by every path** | everything else in `scripts/gazebo/`, the launch files in `src/igvc_test_bringup/launch/`, the robot description, the world. `scripts/gazebo/igvc_env.sh` is what lets one set of scripts run in both layouts |
| Not the simulator | `docker-compose.jetson.yml` and the other services in `docker-compose.yml` are the robot and perception images |

## 5. Maintainers

How the Docker image and the pixi environment are updated, verified and
published, and the rule that keeps the two from drifting apart:
[`docs/MAINTAINING_ENVIRONMENTS.md`](docs/MAINTAINING_ENVIRONMENTS.md).
