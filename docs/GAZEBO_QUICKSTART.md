# Gazebo simulator: set it up and drive the course

**For a team member who has the repo and has never run the simulator.** Follow
it top to bottom and you will end with a robot navigating the IGVC course on
your own machine. Roughly 45 minutes, most of which is one image build you can
walk away from.

`GAZEBO_SETUP.md` is the reference and explains *why* for everything here. This
file is the *how*, and it does not assume you have read that one.

**Verified 2026-09-15** on Windows 11, RTX 5070 Ti Laptop, Docker Desktop
29.7.2, WSL2 Ubuntu-26.04.

---

## 0. The one rule that breaks everything

**Run every `docker` command from a WSL2 Ubuntu shell, never from PowerShell
or CMD.**

Docker Desktop's own Linux VM has no display. A container started from
PowerShell can never open a window - not Gazebo, not RViz - and the failure is
silent: the command appears to work and no window ever arrives. Started from a
WSL2 shell, that distro's WSLg sockets mount through and both windows open as
ordinary Windows windows on the same GPU.

Headless runs work from anywhere. Anything you want to *see* needs WSL2.

Open your WSL2 shell by typing `wsl` in a terminal, or launching "Ubuntu" from
the Start menu. The prompt will end in `$` and paths start `/mnt/c/...`.

---

## 1. Prerequisites

| Need | How to check | If missing |
| --- | --- | --- |
| Windows 11, or Windows 10 build 19044+ | `winver` | Windows 10 below 19044 has no WSLg, so no windows |
| WSL2 with a real distro | `wsl -l -v` shows e.g. `Ubuntu-26.04` with `VERSION 2` | `wsl --install -d Ubuntu` |
| Docker Desktop, WSL2 backend | `docker version` from the WSL2 shell | Install Docker Desktop |
| **WSL Integration enabled for your distro** | see below | see below |
| NVIDIA GPU + recent driver | `nvidia-smi` in PowerShell | Update the driver |
| ~15 GB free disk | | |

### WSL Integration is the one people miss

Docker Desktop -> **Settings** -> **Resources** -> **WSL Integration** ->
enable the toggle for your distro (`Ubuntu-26.04`), then **Apply & Restart**.

Check it worked, from the WSL2 shell:

```bash
docker ps
```

If that errors with "command not found" or "cannot connect", the toggle is off.
Nothing else in this guide will work until it is on.

---

## 1A. Your machine is probably not the one this was built on

Everything here was verified on **one** laptop: Windows 11, RTX 5070 Ti Laptop,
Docker Desktop 29.7.2, WSL2 Ubuntu-26.04. If your machine differs, find it
below **before** you conclude something is broken. Most "it works for you and
not for me" reports on this project have turned out to be one of these.

### Which parts are machine-specific

| Part | Machine-specific? |
| --- | --- |
| Building the image | No. Same everywhere with Docker + internet |
| Running headless, and every check except the GPU one | No |
| **Seeing a window** | **Yes.** Depends on WSLg, or X11 on Linux |
| **GPU rendering** | **Yes.** Depends on vendor, driver and the compose file |
| Speed | Yes. Cameras are the expensive part; see the last table in section 9 |

The simulator **does not require an NVIDIA GPU to run at all.** It requires one
to run *fast* and to make camera results meaningful. If you have no usable GPU,
you can still do navigation, lidar and control work today - see "No usable GPU"
below.

### Windows 11

The verified path. Follow the guide as written.

### Windows 10

Works, with one condition: **WSLg needs build 19044 or newer** (21H2+). Check
with `winver`. Below that you get no windows at all - headless still works, and
so does every check except the RViz one.

Install WSL from the Microsoft Store rather than the older Windows feature, or
`wsl --update`, then confirm:

```powershell
wsl --version
```

You want a `WSLg version` line. No line, no windows.

(Unrelated but worth knowing: Isaac Sim 6.0.1 dropped Windows 10 support
entirely. That affects the Isaac track, not this one. Gazebo is fine.)

### Native Linux

Use the other compose file and the Linux service:

```bash
xhost +local:docker                      # once per login
docker compose up -d igvc_gazebo_linux
docker exec -it igvc_gazebo_linux bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

That service is new as of 2026-09-15 and is **not yet verified on a real Linux
machine** - nobody on the team runs Linux on the desktop. It is the Windows
service with the WSL-specific machinery swapped for `runtime: nvidia`,
`/dev/dri` and `/tmp/.X11-unix`. If it fails, the two usual causes are
`nvidia-container-toolkit` not being installed, and forgetting `xhost`. Please
report back either way so this paragraph can be deleted.

Everything after section 4 is identical, except the container is called
`igvc_gazebo_linux`.

### AMD or Intel GPU

The Windows compose file pins Mesa to the NVIDIA adapter, which is wrong for
you:

```yaml
- MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA    # change or remove
```

On Windows/WSL2, change `NVIDIA` to a string matching your adapter, or delete
the line to let Mesa choose. On native Linux, drop `runtime: nvidia` and the
`NVIDIA_*` variables from `igvc_gazebo_linux` and keep `/dev/dri`.

Then run `render_check.sh` and see what it reports. A modern AMD or Intel GPU
will render this scene perfectly well; the pinning exists only because **this
one laptop has both an Intel iGPU and an NVIDIA card**, and Mesa picking the
Intel one made `gz sim` crash inside Intel's WSL driver. That crash looks like
a Gazebo bug and is not one.

### No usable GPU, or a virtual machine

You will get `SOFTWARE FALLBACK (llvmpipe)`. The simulator still runs. What
changes:

- Turn the cameras off: `CAMERAS=none`. They are what software rendering
  cannot keep up with.
- Expect a real-time factor well below 1.0.
- **Do not take camera or timing measurements** and do not compare them with
  anyone else's.

Navigation, lidar, odometry and control work are all still valid, because
`gpu_lidar` falls back too and the physics is on the CPU regardless. Run:

```bash
docker exec -it igvc_gazebo bash -c \
  "CAMERAS=none NAV=1 HEADLESS=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

### macOS

Not supported. Docker Desktop on macOS gives containers no GPU, and there is no
WSLg equivalent. Software rendering in a VM is the only option and it is not
worth the trouble. Use a lab machine.

### Docker Desktop resource limits

The default allocation is often too small once Gazebo, RViz and ten Nav2 nodes
are running. Settings -> Resources:

| Setting | Minimum | Comfortable |
| --- | --- | --- |
| Memory | 8 GB | 12-16 GB |
| Disk | 15 GB free | 30 GB |
| CPUs | 4 | 8 |

Symptoms of too little memory: containers dying without a message, the build
failing partway, or Nav2's lifecycle bringup aborting with
`Failed to bring up all requested nodes`.

---

## 2. Get the repository

```bash
cd /mnt/c            # or wherever you keep projects
git clone https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
```

If you already have it somewhere with a space in the path, that is fine, but
**quote it everywhere**:

```bash
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
```

---

## 3. Build the image

From the WSL2 shell, in the repo root:

```bash
docker compose -f docker-compose.windows.yml build igvc_gazebo
```

Ten to twenty minutes the first time; it downloads ROS 2 Jazzy, Gazebo Harmonic
and Nav2. The result is `igvc-gazebo-jazzy:latest`, about **5.6 GB**. Later
builds are cached and take seconds unless the Dockerfile changed.

Note the file: **`docker-compose.windows.yml`**, not the plain
`docker-compose.yml`. The plain one targets native Linux and will not work here.

---

## 4. Start the container

```bash
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
```

`up -d` starts it in the background with the fixed name `igvc_gazebo`.

**Do not use `docker compose run`.** It creates a *new* container with a random
name every time, so a second `run` gives you a second, separate simulator
rather than a second shell into the first. That has cost real debugging time:
two simulators on one ROS network both publish `/clock`, `/odom` and `/tf` for
the same robot, and the navigation stack then plans against a robot whose
position jumps between the two. It looks exactly like a navigation bug.

---

## 5. Prove the GPU is actually being used - do this first, every machine

```bash
docker exec -it igvc_gazebo bash -c \
  "source /opt/ros/jazzy/setup.bash && bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

Want:

```text
RESULT: HARDWARE RENDERING (NVIDIA)
```

If you get `SOFTWARE FALLBACK (llvmpipe)`, Gazebo is rendering on the CPU. It
will still run, slowly, and **every camera measurement you take will be
meaningless**. This failure is silent - nothing errors, it is just 20x slower -
which is exactly why this check exists and why it goes first.

Do not use `nvidia-smi` for this. It reports CUDA working fine while OpenGL is
on a software rasteriser.

---

## 6. Run it

```bash
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

Two windows open: **Gazebo** with the IGVC course, and **RViz** showing the
robot, the lidar returns and the front camera.

**Why `src/IGVC_robot_2026/` is in that path.** The container's working
directory is `/root/ros2_ws`, the colcon workspace, and the repo is mounted one
level down at `/root/ros2_ws/src/IGVC_robot_2026`. The directory keeps the 2026
name on purpose: the compose files, `IGVC_WORKSPACE_ROOT` and the DDS profile
all reference it. Drop the prefix and you get
`bash: scripts/gazebo/start_sim.sh: No such file or directory`.

Options, set before the command:

| Variable | Effect |
| --- | --- |
| `NAV=1` | **also start Nav2 and the navigator, so the robot drives itself** |
| `CAMERAS=none` | no cameras; the right choice with software rendering |
| `RVIZ=0` | no RViz window |
| `HEADLESS=1` | no windows at all; for automated checks |
| `CAMERAS=all` | all three ZED cameras instead of just the front one |

So the demo - the robot driving the course by itself:

```bash
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

Give it about 60 seconds. Gazebo loads, then Nav2 starts 15 seconds later on
purpose (see section 9), then the navigator plans and the robot sets off.

**Ctrl-C in that shell stops everything.**

---

## 7. Drive it yourself

Without `NAV=1` nothing is steering the robot, so drive it from a **second
WSL2 shell**, into the same container:

```bash
docker exec -it igvc_gazebo bash
source /root/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` forward, `,` back, `j`/`l` turn, `k` stop. The window must have keyboard
focus.

**The repo's own `teleop.launch.py` does not work here.** It wants a physical
joystick at `/dev/input/js0`, pulls in the ros2_control stack the simulator
does not use, and publishes to a different topic. Use the command above.

Gazebo's DiffDrive holds the last command it was given, so if you drive forward
and let go of the key, the robot keeps going. Press `k`.

---

## 8. Check it actually works

Two scripts. Both run inside the container and both are built to fail when
something is wrong, rather than to look reassuring.

```bash
# the simulator and the topic contract: ~4 minutes
docker exec -it igvc_gazebo bash -c \
  "source /opt/ros/jazzy/setup.bash && bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
```

Expect `topic checks passed: 18   failed: 0`, plus `DRIVE TEST: PASS` and
`FRAME TEST: PASS`. It commands a velocity and checks the odometry moved by
roughly the right amount in roughly the right direction, compared against
Gazebo's own ground truth - because "the topics exist" passes on a badly broken
setup.

```bash
# does the robot drive the course by itself: ~5 minutes
docker exec -it igvc_gazebo bash -c \
  "source /opt/ros/jazzy/setup.bash && bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
```

Expect `AUTONOMY CHECK: PASS`. This script **never publishes a velocity
command** - every command the robot receives comes from Nav2. Reference numbers
from 2026-09-15:

```text
distance travelled   122.9 m
centreline deviation mean 0.72 m, max 1.79 m
AUTONOMY CHECK: PASS
```

Add `RVIZ=1` to either to watch it happen.

---

## 9. When it goes wrong

**"There is no window."** You started it from PowerShell. See section 0.

**`RESULT: SOFTWARE FALLBACK (llvmpipe)`.** The GPU path is broken. Check WSL
Integration is on for your distro, then that `/usr/lib/wsl` exists inside the
container: `docker exec igvc_gazebo ls /usr/lib/wsl/lib`. On a laptop with both
an Intel and an NVIDIA GPU, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` in the
compose file is what stops Mesa picking the Intel one and crashing.

**"A simulator is ALREADY RUNNING in this container."** Exactly what it says,
and the scripts now refuse rather than add a second one. Either let the test
clean up (it does, automatically) or reset:

```bash
docker compose -f docker-compose.windows.yml restart igvc_gazebo
```

**The robot does not move with `NAV=1`.** Give it 60 seconds; Nav2 brings up
ten nodes in sequence. If it still does not, look for
`Failed to bring up all requested nodes` in the output - that is Nav2's
lifecycle bringup losing a race against Gazebo's startup on a loaded machine.
Raise the delay:

```bash
ros2 launch igvc_test_bringup gazebo_nav_test.launch.py nav2_delay:=25.0
```

**Everything is slow.** Check nothing else is running (`docker exec igvc_gazebo
ps -ef | grep gz`), and try `CAMERAS=front`, the default. Three 720p cameras
drop the simulator to about 7 Hz each.

**The build fails partway.** Re-run it first - partial layers are cached and
a network hiccup mid-download is the common cause. If it fails in the same
place twice, check free disk (`df -h` in the WSL2 shell) and Docker Desktop's
disk limit. Behind a corporate proxy or VPN, Docker needs the proxy configured
in Settings -> Resources -> Proxies, or `apt-get` inside the build cannot reach
packages.ros.org.

**`bash: scripts/gazebo/start_sim.sh: cannot execute: required file not
found`.** The repo was checked out with Windows line endings, so the script's
`#!/usr/bin/env bash` ends in a carriage return. `.gitattributes` forces `eol=lf`
for `*.sh`, so this means the checkout predates that or Git was configured to
override it. Fix the checkout rather than adding workarounds:

```bash
git config core.autocrlf false
git rm --cached -r . && git reset --hard
```

**`docker: command not found` in the WSL2 shell.** WSL Integration is off. See
section 1.

**`permission denied while trying to connect to the Docker daemon`.** On native
Linux, add yourself to the `docker` group and log out and back in:
`sudo usermod -aG docker $USER`.

**`no matching manifest` or the image will not pull.** You are on an ARM
machine (Snapdragon laptop, Apple silicon). This image is x86-64 only.

**The container exits immediately after `up -d`.** Check
`docker logs igvc_gazebo`. Usually Docker Desktop is out of memory or the WSL2
VM is out of disk.

**Gazebo opens but the world is empty, or the robot is missing.** The workspace
was not built. `start_sim.sh` builds it for you; if you launched `ros2 launch`
by hand, run the colcon build in section 8 of `GAZEBO_SETUP.md` first.

**Everything is correct and it still does not work.** Capture these four and
post them; they identify almost every cause:

```bash
wsl -l -v                                        # from PowerShell
docker version | head -20
docker exec igvc_gazebo bash -c "source /opt/ros/jazzy/setup.bash && bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh" | tail -5
docker exec igvc_gazebo ps -ef | grep -cE "gz sim|rviz2"
```

---

## 10. What this does and does not do

**It does:** run the real 2026 navigation stack - the ground-truth lane grid,
the IGVC navigator, Nav2's controller, velocity smoother and collision monitor
- against Gazebo, with none of those nodes modified. The robot drives the
course, stays between the lines and goes around the barrels.

**It does not perceive anything.** This is the important one and it is easy to
misreport. The lane lines and the barrel positions come out of
`IGVC_track_generator/track_points.json`, not out of the camera or the lidar.
Both publish, and nothing plans on either. It is a *ground-truth* navigation
test, and that is deliberate: when perception is added, a failure can be blamed
on perception rather than on navigation.

Running the real lane detector here needs two things the image does not have
yet: `torch` and the YOLOPv2 weights.

**It is not a ZED camera.** The simulated cameras publish on the same topics
with the same frame ids and the same intrinsics as the real ZED wrapper, so
downstream code cannot tell the difference. The *content* is a plain pinhole
camera with exact ray-cast depth. ZED neural depth, visual odometry and object
detection have no Gazebo equivalent, and Stereolabs have said they are not
planning to support Gazebo.

**ros2_control is not in the loop.** Nav2's twist goes straight to Gazebo's
built-in DiffDrive, which does the wheel kinematics itself. Joint limits and
the controller update rate are therefore untested in simulation.

---

## Quick reference

```bash
# all of these from a WSL2 shell, in the repo root
docker compose -f docker-compose.windows.yml build igvc_gazebo
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker compose -f docker-compose.windows.yml restart igvc_gazebo   # reset
docker compose -f docker-compose.windows.yml down                  # stop

# inside the container
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh                    # drive it yourself
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"    # it drives itself
docker exec -it igvc_gazebo bash          # a shell, for teleop

# checks, from a shell inside the container (docker exec -it igvc_gazebo bash).
# cwd is /root/ros2_ws; the repo is one level down in src/IGVC_robot_2026.
source /opt/ros/jazzy/setup.bash
bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh          # GPU. run this first, on every machine
bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh    # simulator + topic contract
bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh        # does it drive the course itself
```

| Topic | What |
| --- | --- |
| `/cmd_vel` | velocity in, the only thing flowing into the simulator |
| `/odom` | corrected odometry, world-aligned with its origin at the spawn point |
| `/scan` | RPLidar C1 |
| `/front_zed_camera_x/zed_node/rgb/color/rect/image` | front camera, ZED contract name |
| `/front_zed_camera_x/zed_node/depth/depth_registered` | front depth |
| `/lane_ground_truth`, `/lane_map`, `/lane_costmap` | the ground-truth lane grids |
| `/sim/odom_body_frame` | Gazebo's RAW odometry. Do not use it; see `GAZEBO_SETUP.md` 9.3 |
