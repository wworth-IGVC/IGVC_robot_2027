# Gazebo simulator: set it up and drive the course

**For a team member who has never opened this repository.** Follow it top to
bottom and you will end with a robot navigating the IGVC course on your own
machine.

**Verified 2026-09-17** on Windows 11 Home (build 26200), RTX 5070 Ti Laptop
12 GB, driver 610.88, Docker Desktop 29.7.2 with the WSL2 backend, WSL2
Ubuntu-26.04. Every number in this file was measured on that machine. It is an
x86 laptop, not the robot's Jetson, so do not quote the timings as robot
performance.

`GAZEBO_SETUP.md` is the reference and explains *why* for everything here. This
file is the *how* and does not assume you have read that one.

**How long it takes:**

| Path | Time |
| --- | --- |
| Image handed to you on a USB drive | about 10 minutes |
| Image built from source over good wifi | about 30 minutes |

---

## 0. The one rule that breaks everything

**Run every `docker` command from a WSL2 Ubuntu shell, never from PowerShell
or CMD.**

Docker Desktop's own Linux VM has no display. A container started from
PowerShell can never open a window, not Gazebo and not RViz, and the failure is
**silent**: the command appears to work and no window ever arrives. Started
from a WSL2 shell, that distro's WSLg sockets mount through and both windows
open as ordinary Windows windows on the same GPU.

Headless runs work from anywhere. Anything you want to *see* needs WSL2.

Open your WSL2 shell by typing `wsl` in a terminal, or launching "Ubuntu" from
the Start menu. The prompt ends in `$` and paths start `/mnt/c/...`.

The one exception is `scripts/setup-windows.ps1` in section 2, which only reads
your machine's configuration and is meant for PowerShell.

---

## 1. Prerequisites

| Need | How to check | If missing |
| --- | --- | --- |
| Windows 11, or Windows 10 build 19045+ | `winver` | See "Windows 10" in 1A |
| WSL2 with a real distro | `wsl -l -v` shows e.g. `Ubuntu-26.04`, `VERSION 2` | `wsl --install -d Ubuntu` |
| Docker Desktop, WSL2 backend | `docker version` from the WSL2 shell | Install Docker Desktop |
| **WSL Integration enabled for your distro** | see below | see below |
| NVIDIA GPU + recent driver | `nvidia-smi` in PowerShell | Update the driver. Optional, see 1A |
| Free disk | `df -h /mnt/c` in the WSL2 shell | see the table below |

### Disk, measured rather than guessed

| Item | Measured |
| --- | --- |
| `igvc-gazebo-jazzy` unpacked on disk | **5.57 GB** |
| the same image as a saved tar | **1.20 GB** |
| the clone plus its nine submodules, including `.git` | **0.60 GB** |
| the colcon workspace inside the container | 0.003 GB |
| **total for the USB / offline path** | **7.4 GB** |

**Ask for 12 GB free if you are loading the image from a tar, and 25 GB if you
are building it.** The gap above the measured 7.4 GB is deliberate: the WSL2
VM's `docker_data.vhdx` grows and never shrinks on its own, and one old image
version usually lingers. The 25 GB build figure includes the build cache a
build leaves behind, and that part is **a margin, not a measurement**: this
machine carries 29.06 GB of build cache across four images, 21.86 GB of it
shared, so the share belonging to any single image cannot be cleanly
attributed.

If you are short, reclaim the cheapest thing first:

```bash
docker builder prune          # build cache, safe, usually the biggest win
docker image prune -a         # images nothing is using
```

Be aware that pruning frees space *inside* the VM but `docker_data.vhdx` on
C: stays the same size. Windows 11 **Home** has no Hyper-V, so `Optimize-VHD`
is not available to shrink it.

### WSL Integration is the one people miss

Docker Desktop, **Settings**, **Resources**, **WSL Integration**, enable the
toggle for your distro (`Ubuntu-26.04`), then **Apply & Restart**.

Check it worked, from the WSL2 shell:

```bash
docker ps
```

If that errors with "command not found" or "cannot connect", the toggle is off.
Nothing else in this guide will work until it is on.

---

## 1A. Your machine is probably not the one this was built on

Everything here was verified on **one** laptop. If your machine differs, find
it below **before** you conclude something is broken. Most "it works for you
and not for me" reports on this project have turned out to be one of these.

| Part | Machine-specific? |
| --- | --- |
| Getting the image | No. Same everywhere |
| Running headless, and every check except the GPU one | No |
| **Seeing a window** | **Yes.** Depends on WSLg, or X11 on Linux |
| **GPU rendering** | **Yes.** Depends on vendor, driver and the compose file |
| Speed | Yes. Cameras are the expensive part |

The simulator **does not require an NVIDIA GPU to run at all.** It requires one
to run *fast* and to make camera results meaningful. With no usable GPU you can
still do navigation, lidar and control work today.

### Windows 11

The verified path. Follow the guide as written.

### Windows 10

Works, and one teammate is on it. Three different build numbers matter and they
are easy to conflate:

| Build | What it gates |
| --- | --- |
| 19041 | WSL2 itself |
| **19044** (21H2) | **WSLg, that is, whether a Linux window can ever appear** |
| **19045** (22H2) | **what current Docker Desktop requires** |

Check yours with `winver`.

- **19045 or newer:** supported. But you must install WSL from the **Microsoft
  Store**, not the older "Windows Subsystem for Linux" Windows feature, because
  the feature version has no WSLg and gives you no windows at all. Run
  `wsl --update`, then confirm `wsl --version` prints a **`WSLg version`** line.
  No line, no windows.
- **19044:** you have WSLg but current Docker Desktop will refuse to install.
  **What to do:** update to 22H2 through Settings, Windows Update. It is one
  optional update away and it is the whole fix.
- **19041 to 19043:** this machine can **never** show a Gazebo or RViz window,
  whatever else you install. There is no workaround short of updating Windows.
  **What to do:** update to 22H2. Meanwhile the machine is still useful
  headless: `render_check.sh`, the smoke test and `autonomy_check.sh` all run
  with no display. Pair with someone whose machine shows windows for anything
  visual.
- **Below 19041:** nothing here works. Update Windows first.

`scripts/setup-windows.ps1` prints your build number and which of these you are
in.

(Unrelated but worth knowing: Isaac Sim 6.0.1 dropped Windows 10 support
entirely. That affects the Isaac track, not this one. Gazebo is fine.)

### Native Linux

Use the other compose file and the Linux service:

```bash
xhost +local:docker                      # once per login
docker compose up -d igvc_gazebo_linux
docker exec -it igvc_gazebo_linux bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

That service is **written but has never been run on a real Linux machine**,
because nobody on the team runs Linux on the desktop. It is the Windows service
with the WSL-specific machinery swapped for `runtime: nvidia`, `/dev/dri` and
`/tmp/.X11-unix`. If it fails, the two usual causes are
`nvidia-container-toolkit` not being installed and forgetting `xhost`. Please
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
the line and let Mesa choose. On native Linux, drop `runtime: nvidia` and the
`NVIDIA_*` variables from `igvc_gazebo_linux` and keep `/dev/dri`.

Then run `render_check.sh` and see what it reports. A modern AMD or Intel GPU
will render this scene perfectly well. The pinning exists only because **this
one laptop has both an Intel iGPU and an NVIDIA card**, and Mesa picking the
Intel one made `gz sim` crash inside Intel's WSL driver. That crash looks like
a Gazebo bug and is not one.

### No usable GPU, or a virtual machine

You will get `SOFTWARE FALLBACK (llvmpipe)`. The simulator still runs. What
changes:

- Turn the cameras off: `CAMERAS=none`. They are what software rendering cannot
  keep up with.
- Expect a real-time factor well below 1.0.
- **Do not take camera or timing measurements** and do not compare them with
  anyone else's.

Navigation, lidar, odometry and control work are all still valid, because
`gpu_lidar` falls back too and the physics is on the CPU regardless:

```bash
docker exec -it igvc_gazebo bash -c \
  "CAMERAS=none NAV=1 HEADLESS=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

### macOS

Not supported. Docker Desktop on macOS gives containers no GPU and there is no
WSLg equivalent. Use a lab machine.

### Docker Desktop resource limits

The default allocation is often too small once Gazebo, RViz and ten Nav2 nodes
are running. Settings, Resources:

| Setting | Minimum | Comfortable |
| --- | --- | --- |
| Memory | 8 GB | 12-16 GB |
| CPUs | 4 | 8 |

Symptoms of too little memory: containers dying without a message, the build
failing partway, or Nav2's lifecycle bringup aborting with
`Failed to bring up all requested nodes`.

---

## 2. Get the repository

**The `--recurse-submodules` is not optional.** This is the single most common
way a fresh clone fails, and the failure is confusing when it happens.
`zed_description` is a submodule and colcon builds it by name, so without it
the workspace build fails outright; `IGVC_track_generator` holds the track data
that every navigation node and the world generator read.

From the WSL2 shell:

```bash
cd /mnt/c            # or wherever you keep projects
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
```

Expect nine submodules. Check:

```bash
git submodule status | wc -l          # want: 9
git submodule status | grep '^-'      # want: no output. A leading - means empty
```

**If you already cloned without it**, you do not need to start again:

```bash
git submodule update --init --recursive
```

`bootstrap.sh` in section 4 does this for you and refuses to continue if any
submodule is still empty, so you cannot get far with the wrong clone.

If your path contains a space, that is fine, but **quote it everywhere**:

```bash
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
```

---

## 3. Check your machine, from PowerShell

This one script is meant for PowerShell, because all it does is read your
machine's configuration. It builds nothing and starts nothing.

```powershell
cd "C:\IGVC 2027\IGVC_robot_2027"
.\scripts\setup-windows.ps1
```

It checks the Windows build against all three thresholds above, WSL2, Docker,
GPU passthrough, free disk (printing both the number it wants and the number it
found) and whether your submodules are populated. Every failure comes with what
to do about it.

Expect it to end with:

```text
  All required checks passed.
  Prerequisites are in place. Nothing was built.
```

Switches: `-WithPerception` and `-WithZed` raise the disk requirement for those
much larger images, which the simulator does not need. `-Weights` downloads the
YOLOPv2 model weights, which the simulator does not use.

---

## 4. One command: get the image, build, and verify

Back in the **WSL2** shell, in the repo root:

```bash
bash scripts/gazebo/bootstrap.sh
```

That does everything: submodules, the image, the container, the workspace
build, the GPU check and the smoke test. It prints `PASS` or `FAIL` for each
step and **stops at the first failure** rather than half-succeeding.

**If someone handed you the image on a USB drive, use it.** This skips the
download and the build entirely, which matters when twelve people are on the
same campus wifi. From WSL2, a USB drive mounted at `D:` is `/mnt/d`:

```bash
IMAGE_TAR=/mnt/d/igvc-gazebo-jazzy.tar bash scripts/gazebo/bootstrap.sh
```

Other options:

| Variable | Effect |
| --- | --- |
| `IMAGE_TAR=<path>` | load the image from a tar instead of building it |
| `SKIP_SMOKE=1` | stop after the GPU check. Saves about 4 minutes |
| `FORCE_BUILD=1` | rebuild the image even if it is already present |

Expect, with timings from this machine:

```text
==> step 1: checking where we are and what we have
  PASS  in the repository root
  PASS  running from a WSL2 shell, so GUI windows will work later
  PASS  docker is reachable  (Docker version 29.7.2, build a7dcaa6)
  PASS  free disk: wanted 12 GB, found 112 GB

==> step 2: submodules
  PASS  all 9 submodules populated
  PASS  track data and zed_description are on disk

==> step 3: the image
  PASS  image loaded from the tar

==> step 4: starting the container
  PASS  igvc_gazebo is up

==> step 5: building the workspace
  PASS  four packages built

==> step 6: render_check.sh: is it on the GPU, or silently on the CPU
        GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
        camera topic   : PUBLISHING
        gpu_lidar topic: PUBLISHING
        RESULT: HARDWARE RENDERING (NVIDIA)
  PASS  hardware rendering, and both sensors are publishing

==> step 7: bringup_smoke_test.sh: the topic contract, and does it drive
        topic checks passed: 18   failed: 0
        DRIVE TEST : PASS
        FRAME TEST : PASS
  PASS  the topic contract holds and the robot drives on /cmd_vel

  Bootstrap complete. The simulator is verified on this machine.
```

### If you would rather do the steps by hand

```bash
git submodule update --init --recursive
docker compose -f docker-compose.windows.yml build igvc_gazebo   # or: docker load -i <tar>
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker exec igvc_gazebo bash -c 'source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws &&
  colcon build --symlink-install \
    --base-paths /root/ros2_ws/src/IGVC_robot_2026/src \
    --packages-select zed_description igvc_test_description igvc_test_bringup igvc_lane_detection'
```

Note the compose file: **`docker-compose.windows.yml`**, not the plain
`docker-compose.yml`. The plain one targets native Linux and will not work
here.

**Four packages, not three**, and the fourth is the one people leave out:
`zed_description` is a submodule that `igvc_test_description` depends on, and
omitting it fails the build. (The comment at the top of `start_sim.sh` says
"three"; the code selects four. The code is right.)

**Do not use `docker compose run`.** It creates a *new* container with a random
name every time, so a second `run` gives you a second, separate simulator
rather than a second shell into the first. Two simulators on one ROS domain
both publish `/clock`, `/odom` and `/tf` for the same robot, and the navigation
stack then plans against a robot whose position jumps between them. It looks
exactly like a navigation bug. That is how six `gz sim` processes once ran at
once. Use `up -d` plus `docker exec`.

---

## 5. Making the USB drive, for whoever is handing the image out

Do this once, on a machine that already has the image:

```bash
docker save igvc-gazebo-jazzy:latest -o igvc-gazebo-jazzy.tar
```

| Measured on this machine | |
| --- | --- |
| `docker save` wall time | **14.6 s** |
| resulting file | **1,201,505,280 bytes, 1.20 GB** |
| the image unpacked | 5.57 GB |

The tar is much smaller than the image because it holds the **compressed** layer
blobs, which the daemon unpacks on load. A complete tar is an OCI layout: a
`blobs/sha256/` directory, `index.json`, `manifest.json` and `oci-layout`. You
can check yours is not truncated with `tar -tf igvc-gazebo-jazzy.tar | tail -4`.

Copy that one file to the drive. On the receiving machine, either pass
`IMAGE_TAR` to `bootstrap.sh` as in section 4, or load it by hand:

```bash
docker load -i /mnt/d/igvc-gazebo-jazzy.tar
```

Expect `Loaded image: igvc-gazebo-jazzy:latest`.

**An exFAT drive is required if the image ever grows past 4 GB**, because FAT32
cannot hold a single file that large. 1.20 GB is fine on either, but do not
assume it stays that way.

---

## 6. Run it

```bash
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

Two windows open: **Gazebo** with the IGVC course, and **RViz** showing the
robot, the lidar returns and the front camera.

**Why `src/IGVC_robot_2026/` is in that path.** The container's working
directory is `/root/ros2_ws`, the colcon workspace, and the repo is mounted one
level down at `/root/ros2_ws/src/IGVC_robot_2026`. That directory keeps the
**2026** name on purpose: the compose files, `IGVC_WORKSPACE_ROOT` and the DDS
profile all reference it. Drop the prefix and you get
`bash: scripts/gazebo/start_sim.sh: No such file or directory`.

Options, set before the command:

| Variable | Effect |
| --- | --- |
| `NAV=1` | **also start Nav2 and the navigator, so the robot drives itself** |
| `CAMERAS=none` | no cameras; the right choice with software rendering |
| `CAMERAS=all` | all three ZED cameras instead of just the front one |
| `RVIZ=0` | no RViz window |
| `HEADLESS=1` | no windows at all; for automated checks |

So the demo, the robot driving the course by itself:

```bash
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

Give it about 60 seconds. Gazebo loads, then Nav2 starts 15 seconds later on
purpose (section 9), then the navigator plans and the robot sets off.

**Ctrl-C in that shell stops everything.**

Camera resolution defaults to 640x360 at 15 Hz. Both launch files now accept
`sim_camera_width`, `sim_camera_height` and `sim_camera_hz` if you need to
change it, including on the autonomy path, where until 2026-09-17 they were
silently unreachable:

```bash
ros2 launch igvc_test_bringup gazebo_nav_test.launch.py \
    sim_camera_width:=1280 sim_camera_height:=720
```

`sim_cameras` accepts exactly `none`, `front` or `all`. Anything else is now
rejected with a readable error. It used to silently build a robot with **zero
cameras** that launched cleanly.

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
joystick at `/dev/input/js0`, pulls in the ros2_control stack the simulator does
not use, and publishes to a different topic. Use the command above.

Gazebo's DiffDrive **holds the last command it was given**, so if you drive
forward and let go of the key, the robot keeps going. Press `k`. Measured
today: it travelled a further **3.672 m in the 4 s after `/cmd_vel` stopped**.
Whether that is a true absence of timeout or a long one is still an open
question, and `diff_drive_controller` on the real robot does time out, so the
simulator is the less safe of the two.

---

## 8. Check it actually works

Both run inside the container and both are built to fail when something is
wrong rather than to look reassuring. `bootstrap.sh` runs the first two for
you; these are the commands to re-run one on its own.

```bash
docker exec -it igvc_gazebo bash -c \
  "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

Want `RESULT: HARDWARE RENDERING (NVIDIA)`.

**You no longer need to source ROS before this.** Earlier versions of this
guide told you to, and until 2026-09-17 you genuinely had to, because `gz` on
this image comes from the ROS *vendor* packages and `/root/.bashrc` does not
source `setup.bash`. The script failed on a clean container with `gz: No such
file or directory` and printed `RESULT: UNKNOWN`, which reads as a broken GPU
rather than a missing PATH entry. It sources ROS itself now.

```bash
# the simulator and the topic contract: about 4 minutes
docker exec -it igvc_gazebo bash -c \
  "bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
```

Measured on this machine today:

```text
topic checks passed: 18   failed: 0
odometry displacement : 7.073 m
simulator ground truth: 7.081 m
heading disagreement  : 0.13 deg     (a wrong odom frame shows up here)
distance ratio        : 0.9988       (a wrong wheel radius shows up here)
DRIVE TEST : PASS
FRAME TEST : PASS
```

It commands a velocity and checks the odometry moved by roughly the right
amount in roughly the right direction, compared against Gazebo's own ground
truth, because "the topics exist" passes on a badly broken setup.

```bash
# does the robot drive the course by itself: about 5 minutes
docker exec -it igvc_gazebo bash -c \
  "bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
```

This script **never publishes a velocity command**: every command the robot
receives comes from Nav2. Measured today:

```text
samples              : 893  at 9.5 Hz
distance travelled   : 83.0 m
furthest two points  : 29.3 m apart
centreline deviation : mean 0.67 m, max 1.86 m
yaw to lane          : mean 15.9 deg, max 48.0 deg
lane clearance       : worst +0.040 m (chassis), +0.081 m (Nav2 footprint)
                       0.0% of samples over the line

MOVED       : PASS      PROGRESS    : PASS
IN LANE     : PASS      ON THE SLAB : PASS
AUTONOMY CHECK: PASS
```

**Read `IN LANE` with care. It is currently a coin flip and that is a known
defect in the test, not in the robot.** It compares centreline deviation
against a flat 2.00 m tolerance, but the lane width varies by a factor of two
along the course, so the threshold is measuring the wrong quantity and the runs
sit right on it: three runs at honest sampling produced 2.04 (FAIL), 1.95
(PASS) and 1.98 (PASS). In every one of those runs the robot drove 47 to 51 m
under its own control, stayed on the slab, and put **0.0%** of samples over the
paint. If you get a FAIL here, look at the clearance line before believing the
robot misbehaved. A team decision is pending and nobody should raise the
tolerance to make it green.

Add `RVIZ=1` to either to watch it happen.

---

## 9. When it goes wrong

**"There is no window."** You started it from PowerShell. See section 0.

**`RESULT: SOFTWARE FALLBACK (llvmpipe)`.** The GPU path is broken. Check WSL
Integration is on for your distro, then that `/usr/lib/wsl` exists inside the
container: `docker exec igvc_gazebo ls /usr/lib/wsl/lib`. On a laptop with both
an Intel and an NVIDIA GPU, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` in the
compose file is what stops Mesa picking the Intel one and crashing. Do **not**
use `nvidia-smi` to check this: it reports CUDA working fine while OpenGL is on
a software rasteriser.

**`colcon build` fails with a package it cannot find, usually
`zed_description`.** Your submodules are empty. `git submodule update --init
--recursive`. See section 2.

**"A simulator is ALREADY RUNNING in this container."** Exactly what it says,
and the scripts refuse rather than add a second one. Either let the test clean
up, which it does automatically, or reset:

```bash
docker compose -f docker-compose.windows.yml restart igvc_gazebo
```

**The robot does not move with `NAV=1`.** Give it 60 seconds; Nav2 brings up
ten lifecycle nodes in sequence. If it still does not, look for `Failed to
bring up all requested nodes` in the output. That is Nav2's lifecycle bringup
losing a race against Gazebo's startup on a loaded machine. Raise the delay:

```bash
ros2 launch igvc_test_bringup gazebo_nav_test.launch.py nav2_delay:=25.0
```

**Everything is slow.** Check nothing else is running
(`docker exec igvc_gazebo ps -ef | grep gz`), and use `CAMERAS=front`, the
default. Three 720p cameras drop the simulator to about 7 Hz each. Note that
`real_time_factor` will read near 1.0 anyway, because gz-sim runs physics and
rendering on separate threads. Count messages, never trust RTF.

**The build fails partway.** Re-run it first: partial layers are cached and a
network hiccup mid-download is the common cause. If it fails in the same place
twice, check free disk (`df -h` in the WSL2 shell) and Docker Desktop's disk
limit. Behind a corporate proxy or VPN, Docker needs the proxy configured in
Settings, Resources, Proxies, or `apt-get` inside the build cannot reach
packages.ros.org.

**`bash: scripts/gazebo/start_sim.sh: cannot execute: required file not
found`.** The repo was checked out with Windows line endings, so the script's
`#!/usr/bin/env bash` ends in a carriage return. `.gitattributes` forces
`eol=lf` for `*.sh`, so a normal clone is fine; this means Git was configured
to override it. Fix the checkout rather than adding workarounds:

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
was not built. `bootstrap.sh` and `start_sim.sh` both build it for you; if you
launched `ros2 launch` by hand, run the colcon build in section 4 first.

**Everything is correct and it still does not work.** Capture these four and
post them; they identify almost every cause:

```bash
wsl -l -v                                        # from PowerShell
docker version | head -20
docker exec igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh" | tail -5
docker exec igvc_gazebo ps -ef | grep -cE "gz sim|rviz2"
```

---

## 10. What this does and does not do

**It does:** run the real 2026 navigation stack, that is the ground-truth lane
grid, the IGVC navigator, Nav2's controller, velocity smoother and collision
monitor, against Gazebo, with none of those nodes modified. The robot drives
the course, stays between the lines and goes around the barrels.

**It does not perceive anything.** This is the important one and it is easy to
misreport. The lane lines and the barrel positions come out of
`IGVC_track_generator/track_points.json`, not out of the camera or the lidar.
The camera, depth image, point cloud and lidar all publish, and **nothing plans
on any of them**. It is a *ground-truth* navigation test, and that is
deliberate: when perception is added, a failure can be blamed on perception
rather than on navigation.

More precisely, `gazebo_nav_test_nav2_overrides.yaml` replaces the costmap
`plugins` list with `["lane_layer", "inflation_layer"]`, so Nav2's
`obstacle_layer` is never even instantiated. Every obstacle the robot has ever
avoided in this simulator came from the JSON.

**The Hough lane detector does run**, and needs nothing extra: it is CLAHE,
Canny and `HoughLinesP` with no torch, and it produced 253 occupied cells in
`/lane_map` over a 90 s self-driven run. Nothing plans on its output.
**YOLOPv2 (`lane_segmentation_node`) does not run here**: it needs torch, which
this image does not have, and weights that are not in the repo.

**The camera point cloud is rotated 90 degrees.** Measured, not suspected: the
cloud carries body-convention data stamped with the optical frame, so through
its own frame the floor stands on its end. It is bridged, nothing consumes it,
and `obstacle_layer` is off, so it cannot currently do harm. **It is not
fixed**, deliberately, because fixing it means connecting a Nav2 consumer that
has never been connected. See `GAZEBO_SETUP.md` 10.6 and 10.10.

**`/fix` is not published.** The GPS row of the interface contract is the one
unbridged row. Nothing consumes it today.

**It is not a ZED camera.** The simulated cameras publish on the same topics
with the same frame ids and the same intrinsics as the real ZED wrapper, so
downstream code cannot tell the difference. The *content* is a plain pinhole
camera with exact ray-cast depth. ZED neural depth, visual odometry and object
detection have no Gazebo equivalent, and Stereolabs have said they are not
planning to support Gazebo. The simulated field of view is 100 degrees and
**which lens is actually fitted to the robot is unknown**, so the sim is either
4.6 degrees too narrow or 26 degrees too wide and the sign is not known.

**ros2_control is not in the loop.** Nav2's twist goes straight to Gazebo's
built-in DiffDrive, which does the wheel kinematics itself. Joint limits and
the controller update rate are therefore untested in simulation.

**Nothing here has been run on any machine but the one at the top of this
file.** Not the RTX 5080 laptop, not the Windows 10 machine, not native Linux.

---

## Quick reference

```bash
# all of these from a WSL2 shell, in the repo root
bash scripts/gazebo/bootstrap.sh                                   # zero to verified
IMAGE_TAR=/mnt/d/igvc-gazebo-jazzy.tar bash scripts/gazebo/bootstrap.sh   # from USB

docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker compose -f docker-compose.windows.yml restart igvc_gazebo   # reset
docker compose -f docker-compose.windows.yml down                  # stop

docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh                  # drive it yourself
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"  # it drives itself
docker exec -it igvc_gazebo bash                                   # a shell, for teleop

# the three checks. cwd inside the container is /root/ros2_ws;
# the repo is one level down in src/IGVC_robot_2026.
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
```

| Topic | What |
| --- | --- |
| `/cmd_vel` | velocity in, the only thing flowing into the simulator |
| `/odom` | corrected odometry, world-aligned, origin at the spawn point |
| `/scan` | RPLidar C1 |
| `/front_zed_camera_x/zed_node/rgb/color/rect/image` | front camera, ZED contract name |
| `/front_zed_camera_x/zed_node/depth/depth_registered` | front depth |
| `/lane_ground_truth`, `/lane_map`, `/lane_costmap` | the ground-truth lane grids |
| `/sim/odom_body_frame` | Gazebo's RAW odometry. Do not use it; see `GAZEBO_SETUP.md` 9.3 |
