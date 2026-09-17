# Gazebo simulator: from a blank Windows install to driving the course

**Assumes nothing is installed.** No Git, no Docker, no WSL2, no Python. If you
have a fresh Windows laptop and about 60 GB free on C:, this file is everything
you need.

**Verified 2026-09-17** on Windows 11 Home build 26200, RTX 5070 Ti Laptop
12 GB, driver 610.88, Docker Desktop 29.7.2 with the WSL2 backend, WSL2
Ubuntu-26.04. Every number below was measured on that one machine. It is an
x86 laptop, **not** the robot's Jetson, so do not quote the timings as robot
performance.

`GAZEBO_SETUP.md` is the reference and explains *why* for everything here.
This file is the *how*.

---

## Start here: the decision tree

You do not need to read three paths to find yours. Read this, then jump.

```text
  1. Do you have a Mac?
        yes -> section 5. Short answer: tonight you pair with a Windows
               laptop. A Mac can run the headless checks under emulation,
               but it CANNOT show the Gazebo window at all: XQuartz's GL
               is too old for gz-sim's renderer. Do not spend tonight on it.
        no  -> continue

  2. Do you have Administrator rights on this laptop?
        no  -> you cannot install Docker Desktop. Pair with someone.
               See section 1, check 1.
        yes -> continue

  3. Work through section 1 (triage), then section 2 (installs),
     then section 3 (the repo), then section 4 (one command).

  4. Section 4 runs render_check.sh, which PRINTS YOUR TIER.
     Read it off the screen. Then:

        YOUR TIER: A   discrete NVIDIA        -> section 6. Everything works.
        YOUR TIER: B   integrated Intel/AMD   -> section 6, same as A, but
                                                 your performance is unmeasured.
        YOUR TIER: C   software rendering     -> section 7. Still useful.
                                                 Headless plus RViz.
        YOUR TIER: unknown                    -> treat as C, report the adapter.
```

**Nobody is excluded.** Tier C is a supported configuration, not a failure. The
physics runs on the CPU either way and the lidar falls back too, so navigation,
odometry and control work are all still valid on a laptop with no usable GPU.
Cameras are the only part that really suffers.

---

## 1. Triage, before you install anything

Three checks. Each has a consequence, so do them first rather than discovering
the problem at minute seventy.

### Check 1: do you have Administrator rights?

Open **PowerShell** and run:

```powershell
([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
```

`True` means this shell is elevated. If you get `False`, right-click PowerShell
and choose **Run as administrator**, then run it again.

**If you cannot get `True` at all, stop here.** Docker Desktop needs
administrator rights to install, and a university-managed laptop may simply
refuse. **Consequence: you pair with someone else for this session.** That is
a normal outcome and not worth fighting during a two-hour meeting; raise it
with IT afterwards.

### Check 2: is your Windows new enough?

```powershell
[System.Environment]::OSVersion.Version
```

Or just run `winver`. You want the **build** number.

| Build | What it means |
| --- | --- |
| 22000 or higher | **Windows 11. You are fine.** Skip the rest of this check |
| 19045 or higher | Windows 10 22H2. Supported, but see the note below |
| 19044 | Windows 10 21H2. You have WSLg, but current Docker Desktop will refuse to install |
| 19041 to 19043 | Too old for WSLg. **No Linux window can ever appear on this machine** |
| below 19041 | Too old for WSL2 at all |

**On Windows 10 you must install WSL from the Microsoft Store**, not the older
"Windows Subsystem for Linux" Windows feature. The feature version has no WSLg
and gives you no windows at all. `wsl --update` does this for you. Confirm
afterwards that `wsl --version` prints a **`WSLg version`** line. No line, no
windows.

**The exact message to show if your build is too old**, so nobody argues with
it:

```text
  Windows build <yours> is below 19044.
  WSLg does not exist on this build, so no Gazebo or RViz window can
  appear on this machine, no matter what else is installed. There is no
  workaround short of updating Windows.

  What to do: Settings > Windows Update, update to 22H2. That is one
  optional update away.

  Until then this machine can still run everything headless:
  render_check.sh, bringup_smoke_test.sh and autonomy_check.sh all work
  with no display. Pair with someone whose machine shows windows for
  anything visual.
```

### Check 3: which GPU do you have?

This decides your tier in the table above. In **PowerShell**:

```powershell
Get-CimInstance Win32_VideoController | Select-Object Name, DriverVersion
```

Or run `dxdiag`, wait for it to finish, and read the **Display** tabs.

| What you see | Likely tier |
| --- | --- |
| An NVIDIA GeForce, RTX, Quadro or Tesla adapter | **A** |
| Only Intel (UHD, Iris, Arc) or only AMD (Radeon, Vega) | **B** |
| Both an Intel iGPU **and** an NVIDIA card | **A**, but see the tier B note in section 6 about the adapter crash |
| Nothing recognisable, or a virtual adapter | **C** |

This is a prediction, not the answer. `render_check.sh` in section 4 gives you
the real one, because what matters is whether the GPU reaches the *container*,
not whether Windows can see it.

---

## 2. Installs, on a blank Windows machine

**Every command in this section is PowerShell, running as Administrator.**
Nothing in this section belongs in a WSL2 shell, because the WSL2 shell does
not exist yet.

`winget` ships with Windows 11 and is the fastest path, with no browser
hunting. Check it exists:

```powershell
winget --version
```

Verified on this machine: `v1.29.290`. If `winget` is not found, you are
probably on Windows 10 where it arrives via an App Installer update: install
**App Installer** from the Microsoft Store, or get the msixbundle from
`https://github.com/microsoft/winget-cli/releases`. Every step below also has a
manual download link, so a missing or blocked `winget` is an inconvenience and
not a blocker.

Add `--accept-source-agreements` on the first `winget` command of a fresh
machine, or it stops to ask.

### 2.1 Git for Windows

```powershell
winget install --id Git.Git --exact --source winget --accept-source-agreements
```

Verified: `Git.Git`, version 2.55.0.3, source `winget`.
Manual download: `https://gitforwindows.org/`

**Do not install `Microsoft.Git`.** That Id is real but it is Microsoft's fork
build, not the standard Git for Windows installer this project expects.

**Then set the line-ending behaviour, and this one matters.** This repository's
shell scripts must stay LF. If Git rewrites them to CRLF, every script in the
repo fails inside the container with `bad interpreter: /usr/bin/env bash^M`,
which reads as a broken script rather than a broken checkout:

```powershell
git config --global core.autocrlf input
```

`.gitattributes` in this repo already forces `eol=lf` for `*.sh`, so a normal
clone is safe either way. Setting this makes it safe even if something else on
your machine has opinions.

**Verify:**

```powershell
git --version
git config --global core.autocrlf
```

Expect `git version 2.55.0.windows.3` or similar, then `input`.

### 2.2 WSL2 and Ubuntu

**Use `wsl --install`, not winget.** WSL does exist in winget as
`Microsoft.WSL`, but `wsl --install` is the documented path because it also
enables the Windows optional features (Virtual Machine Platform, and WSL
itself) that an app install does not.

```powershell
wsl --install
```

**>>> THIS NEEDS A REBOOT. <<<** Do it now, not later. A restart at minute
seventy kills the session.

```powershell
Restart-Computer
```

After the reboot, WSL finishes setting up and Ubuntu launches on its own. If it
does not, run `wsl --install -d Ubuntu`.

**>>> Ubuntu will ask you to create a UNIX username and password. <<<** This is
a new account inside Linux and has nothing to do with your Windows login.
Type a short lowercase username. **The password does not echo as you type, not
even dots. That is normal, keep typing.** Write the password down; you need it
for `sudo`.

Then update WSL itself, because WSLg fixes ship here:

```powershell
wsl --update
```

**Verify, from PowerShell:**

```powershell
wsl --version
wsl -l -v
```

`wsl --version` must print a **`WSLg version`** line. No WSLg line means no
GUI, ever. `wsl -l -v` must show your distro with **`VERSION 2`**, for example:

```text
  NAME              STATE           VERSION
* Ubuntu-26.04      Running         2
```

If it says `VERSION 1`, convert it: `wsl --set-version Ubuntu 2`.

Manual instructions if `wsl --install` fails:
`https://learn.microsoft.com/windows/wsl/install-manual`

### 2.3 Docker Desktop

```powershell
winget install --id Docker.DockerDesktop --exact --source winget
```

Verified: `Docker.DockerDesktop`, version 4.91.0, source `winget`.
Manual download: `https://www.docker.com/products/docker-desktop`

**>>> THIS NEEDS A REBOOT, OR AT LEAST A LOGOUT. <<<** Docker Desktop adds you
to a local group and the change does not apply to an existing session. Reboot.

Then **launch Docker Desktop from the Start menu** and accept the terms. It
does not start by itself after installation. Wait for the whale icon in the
system tray to stop animating.

**>>> Now the UI step this project has already been bitten by. <<<** It is not
a command and it cannot be scripted:

1. Open **Docker Desktop**
2. **Settings** (the gear icon)
3. **General**: confirm **Use the WSL 2 based engine** is ticked
4. **Resources** > **WSL Integration**
5. Turn on the toggle for your distro, for example `Ubuntu-26.04`
6. **Apply & Restart**

Without step 5, `docker` exists in PowerShell and **does not exist in the WSL2
shell**, which is where every later command runs.

**Verify, and do it in BOTH shells, because passing in one proves nothing about
the other:**

```powershell
# PowerShell
docker version
docker run hello-world
```

```bash
# now in the WSL2 Ubuntu shell: type  wsl  in a terminal, or launch Ubuntu
docker version
docker run hello-world
```

Both must print `Hello from Docker!`. If the WSL2 one says
`docker: command not found` or `cannot connect`, WSL Integration is off. Go
back to step 5.

### 2.4 Anything else?

**Nothing.** You do not need Python, `make`, a compiler or an editor on
Windows. ROS 2, Gazebo, `colcon` and `python3` all live **inside the
container**, and `git`, `bash` and `df` come with WSL2 Ubuntu. If a document
tells you to install Python for this path, it is out of date.

### 2.5 Time and bandwidth, measured

| Step | Download | Time on this machine |
| --- | --- | --- |
| Git for Windows | about 65 MB | 1 to 2 min |
| `wsl --install` plus Ubuntu | about 500 MB | 3 to 5 min, plus a reboot |
| `wsl --update` | about 130 MB | under 1 min |
| Docker Desktop | about 600 MB installer | 5 to 8 min, plus a reboot |
| the repo clone with 9 submodules | about 600 MB | **17.4 s** measured |
| the Gazebo image, pulled | about 1.2 GB over the wire, 5.57 GB unpacked | see section 4 |

**Rough room total: about 2.4 GB per laptop, so five laptops is on the order of
12 GB.** On shared campus wifi that is the binding constraint of the evening,
which is why downloads start at minute zero and why pulls are staggered rather
than all five at once.

**Disk, measured rather than guessed:**

| Item | Measured |
| --- | --- |
| the Gazebo image, unpacked on disk | **5.57 GB** |
| the clone plus its nine submodules, including `.git` | **0.60 GB** |
| the colcon workspace inside the container | 0.003 GB |
| Docker Desktop plus the WSL2 Ubuntu distro | roughly 4 GB |

**Ask for 20 GB free, and 30 GB if you build the image instead of pulling it.**
You were asked to clear 60 GB, which is comfortable headroom and deliberately
so: `docker_data.vhdx` grows and never shrinks, and Windows 11 **Home** has no
Hyper-V, so `Optimize-VHD` is not available to compact it. Space you give
Docker does not come back easily.

If you are short:

```bash
docker builder prune          # build cache, safe, usually the biggest win
docker image prune -a         # images nothing is using
```

---

## 3. Get the repository

**From here on, every command is in the WSL2 Ubuntu shell**, not PowerShell.
Type `wsl` in a terminal, or launch Ubuntu from the Start menu. The prompt ends
in `$` and paths start `/mnt/c/...`.

**Why it matters:** Docker Desktop's own Linux VM has no display. A container
started from PowerShell can never open a window, not Gazebo and not RViz, and
**the failure is silent**: the command appears to work and no window ever
arrives. Started from a WSL2 shell, that distro's WSLg sockets mount through
and both windows open as ordinary Windows windows on the same GPU.

Headless runs work from anywhere. Anything you want to *see* needs WSL2.

```bash
cd /mnt/c
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
```

**The `--recurse-submodules` is not optional.** There are nine submodules and
two are load-bearing: `zed_description` is built by name, so the workspace
build fails outright without it, and `IGVC_track_generator` holds the track
data that every navigation node and the world generator read.

**Verify:**

```bash
git submodule status | wc -l          # want: 9
git submodule status | grep '^-'      # want: no output. A leading - means empty
```

If you already cloned without it, you do not need to start again:

```bash
git submodule update --init --recursive
```

Section 4 refuses to continue if any submodule is empty, so you cannot get far
with the wrong clone.

---

## 4. One command: get the image, build, and find your tier

```bash
bash scripts/gazebo/bootstrap.sh
```

That does everything: submodules, the image, the container, the workspace
build, the GPU tier check and the smoke test. It prints `PASS` or `FAIL` for
each step and **stops at the first failure** rather than half-succeeding.

The image comes from **GitHub Container Registry**. It is a public package, so
**no `docker login` is needed**: about 1.2 GB over the wire, unpacking to
5.57 GB on disk.

| Variable | Effect |
| --- | --- |
| `BUILD_IMAGE=1` | build from the Dockerfile instead of pulling. The fallback for when GHCR is unreachable. 10 to 20 minutes instead of a few |
| `IMAGE_REF=...` | pull a different tag, for example the dated `ghcr.io/wworth-igvc/igvc-gazebo-jazzy:2026-09-17` instead of `latest` |
| `SKIP_SMOKE=1` | stop after the tier check. Saves about 4 minutes |
| `ALLOW_TIER_C=1` | do not pause to explain a software-rendering machine |

Expect, on tier A:

```text
==> step 1: checking where we are and what we have
  PASS  in the repository root
  PASS  running from a WSL2 shell, so GUI windows will work later
  PASS  docker is reachable  (Docker version 29.7.2, build a7dcaa6)
  PASS  free disk:  WANTS 20 GB   FOUND 106 GB

==> step 2: submodules
  PASS  all 9 submodules populated
  PASS  track data and zed_description are on disk

==> step 3: the image
        pulling ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
  PASS  image pulled and tagged as igvc-gazebo-jazzy:latest

==> step 4: starting the container
  PASS  igvc_gazebo is up

==> step 5: building the workspace
  PASS  four packages built

==> step 6: render_check.sh: which GPU tier is this machine
        ADAPTER: D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
        RESULT: HARDWARE RENDERING (NVIDIA)
        YOUR TIER: A
        camera topic   : PUBLISHING
        gpu_lidar topic: PUBLISHING
  PASS  TIER A, hardware rendering on a discrete NVIDIA adapter

==> step 7: bringup_smoke_test.sh: the topic contract, and does it drive
        topic checks passed: 18   failed: 0
        DRIVE TEST : PASS   robot moved 7.331 m on /cmd_vel
        FRAME TEST : PASS   odometry agrees with the simulator
  PASS  the topic contract holds and the robot drives on /cmd_vel

  Bootstrap complete. The simulator is verified on this machine.
  Your GPU tier: A
```

**Now read your tier off that output and go to section 6 (A or B) or section 7
(C).**

`bootstrap.sh` will not tell you the GPU works when it does not. It reads the
tier from `render_check.sh` rather than guessing, and it fails outright if the
two ever disagree.

### If you would rather do the steps by hand

```bash
git submodule update --init --recursive
docker pull ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
docker tag ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest igvc-gazebo-jazzy:latest
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker exec igvc_gazebo bash -c 'source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws &&
  colcon build --symlink-install \
    --base-paths /root/ros2_ws/src/IGVC_robot_2026/src \
    --packages-select zed_description igvc_test_description igvc_test_bringup igvc_lane_detection'
```

Note the compose file: **`docker-compose.windows.yml`**, not the plain
`docker-compose.yml`, which targets native Linux and will not work here.

**Four packages, not three**, and the fourth is the one people leave out:
`zed_description` is a submodule that `igvc_test_description` depends on, and
omitting it fails the build.

**Do not use `docker compose run`.** It creates a *new* container with a random
name every time, so a second `run` gives you a second, separate simulator
rather than a second shell into the first. Two simulators on one ROS domain
both publish `/clock`, `/odom` and `/tf` for the same robot, so the navigation
stack plans against a robot whose position jumps between them. It looks exactly
like a navigation bug, and it is how six `gz sim` processes once ran at once.
Use `up -d` plus `docker exec`.

### If GHCR is unreachable in the room

Two fallbacks, in order:

1. `BUILD_IMAGE=1 bash scripts/gazebo/bootstrap.sh`, which builds from the
   Dockerfile. 10 to 20 minutes and it needs `packages.ros.org`, so it is not
   an offline path.
2. Ask Liam to serve the image off his laptop over the local network. See the
   LAN fallback section of the session runbook.

---

## 5. macOS

**Short answer: tonight, pair with a Windows laptop on tier A or B.**

A Mac is not excluded in principle, and the headless checks should run. But
the one thing people assume will work does not, so read the third point before
you spend the evening on it. Each finding is labelled with how well it is
established.

**1. Docker Desktop on macOS does not expose the GPU to Linux containers.
VERIFIED.** Docker's own GPU documentation says flatly that "GPU support in
Docker Desktop is only available on Windows with the WSL2 backend". The cause
is architectural, not a setting: containers run inside a Linux VM on Apple's
Virtualization framework, which gives Linux guests no 3D-capable virtual GPU.
There is no `--gpus` equivalent to try. **Consequence: a Mac is tier C at
best**, on hardware that is otherwise very capable.

**2. On Apple Silicon this amd64 image runs under emulation. VERIFIED, but
"Rosetta" is wrong. REFUTED.** Our image is amd64 only, and Docker's docs
confirm you cannot run a `linux/amd64` container on an arm64 host without
emulation. However **Rosetta is optional and off by default**: Docker's
settings reference lists "Use Rosetta for x86_64/amd64 emulation on Apple
Silicon" as *Disabled* by default. The default path is QEMU via
`binfmt_misc`, which Docker's own docs warn "can be much slower than native
builds, especially for compute-heavy tasks", and CPU rasterisation is exactly
compute-heavy. Also **`--platform linux/amd64` is good practice rather than a
hard requirement**: with only an amd64 manifest there is nothing for Docker to
choose, so it starts the container under emulation and prints a
platform-mismatch warning. Set `platform: linux/amd64` in compose anyway so
the behaviour is predictable instead of warning-driven.

**3. XQuartz cannot show you the Gazebo GUI. REFUTED, and this is the finding
that decides tonight.** The usual advice is "install XQuartz, set `DISPLAY`,
run `xhost`". For most Linux GUI apps that works. **For Gazebo it does not.**
XQuartz's GLX caps out at roughly **OpenGL 1.4**, and gz-sim's default `ogre2`
render engine requires **OpenGL above 3.3, preferably 4.3+**. Enabling indirect
GLX runs the GL calls against that 1.4 stack, so `gz sim` fails with an ogre2
OpenGL error rather than rendering slowly. There is no WSLg equivalent on macOS
either (ASSUMED: it is an absence claim, so no single document asserts it).

If someone does want a visual path on a Mac later, the route that works is
**not** XQuartz but an X server plus VNC or noVNC *inside* the container,
viewed in a browser: that rasterises client-side with llvmpipe and never
touches XQuartz's GL at all. Nobody has done it here.

**So what a Mac can actually do, ASSUMED because nobody on this team has
tested it on a Mac:** the headless checks. `bringup_smoke_test.sh` and
`autonomy_check.sh` should both run under emulation, because the autonomy path
takes its lane lines and barrels from `track_points.json` rather than from the
camera, so software rendering does not invalidate the result. What will not
work is the Gazebo GUI and any camera-based perception number. Two cautions:
an emulated run may miss timing-sensitive checks, and **the open DiffDrive
`/cmd_vel` timeout question must not be investigated on a Mac**, because that
measurement has already been retracted three times for less.

**Backlog, scoped and deliberately not started: a native arm64 image is
feasible, and more so than expected.** Verified today by downloading the real
apt indexes rather than reading documentation:

- REP-2000 lists Ubuntu Noble 24.04 **arm64 as a Tier 1 platform for Jazzy**.
- `packages.ros.org/ros2/ubuntu/dists/noble/Release` declares
  `Architectures: i386 amd64 arm64 armhf`.
- The arm64 package index carries **3667 `ros-jazzy-*` packages**, including
  every one this image installs: `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-sim`,
  `ros-jazzy-ros-gz-bridge` and the rest.
- This image needs **no ZED SDK**, which is the usual arm64 blocker.

A multi-arch build would give Macs a hardware-native, software-rendered path
with no emulation layer underneath it. **Estimate: half a day**, mostly a
`docker buildx` multi-arch pipeline plus one round of verification on an actual
Mac. Do not start it before the meeting.

---

## 6. Tier A and tier B: run it

```bash
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

Two windows open: **Gazebo** with the IGVC course, and **RViz** showing the
robot, the lidar returns and the front camera.

**Why `src/IGVC_robot_2026/` is in that path.** The container's working
directory is `/root/ros2_ws`, the colcon workspace, and the repo is mounted one
level down at `/root/ros2_ws/src/IGVC_robot_2026`. That directory keeps the
**2026** name on purpose: the compose files, `IGVC_WORKSPACE_ROOT` and the DDS
profile all reference it.

Options, set before the command:

| Variable | Effect |
| --- | --- |
| `NAV=1` | **also start Nav2 and the navigator, so the robot drives itself** |
| `CAMERAS=none` | no cameras |
| `CAMERAS=all` | all three ZED cameras instead of just the front one |
| `RVIZ=0` | no RViz window |
| `HEADLESS=1` | no windows at all; for automated checks |

The demo, the robot driving the course by itself:

```bash
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

Give it about 60 seconds. Gazebo loads, Nav2 starts 15 seconds later on
purpose, then the navigator plans and the robot sets off. **Ctrl-C in that
shell stops everything.**

Camera resolution defaults to 640x360 at 15 Hz. Both launch files accept
`sim_camera_width`, `sim_camera_height` and `sim_camera_hz`.

### The tier B note: the adapter crash

Tier B means your GPU **is** reaching the container and rendering in hardware.
That is a real pass. **Tier B is not measured by this project on any machine**,
so treat performance as unknown rather than assuming it matches tier A, and
please report your numbers back.

One known failure mode applies if your laptop has **both** an Intel iGPU and a
discrete card. `docker-compose.windows.yml` contains:

```yaml
- MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA    # change or remove
```

That line exists because on the machine this was built on, Mesa picking the
Intel adapter made `gz sim` crash inside Intel's WSL driver. The crash looks
like a Gazebo bug and is not one. If you have no NVIDIA adapter, change
`NVIDIA` to a string matching yours or delete the line and let Mesa choose,
then re-run `render_check.sh`.

---

## 7. Tier C: software rendering, and still useful

You got `RESULT: SOFTWARE RENDERING` and `YOUR TIER: C`. **Nothing is wrong
with your machine.** The GPU is not reaching the container, so Mesa rasterised
on the CPU.

**What still works, and it is most of the project:** the physics runs on the
CPU regardless, and `gpu_lidar` falls back to software too, so **navigation,
lidar, odometry, control and the whole Nav2 stack are all valid work.**

**What does not:** camera throughput, and therefore anything perception. **Do
not take camera or timing measurements on this configuration and do not compare
them with anyone else's.**

### The tier C recipe

**Run headless with RViz. Do not open the Gazebo window.** The Gazebo GUI is
itself a rendered 3D view, so on a software rasteriser it competes with the
simulation for the same CPU. RViz is far cheaper and shows you what matters:
the robot, the lidar returns, the odometry trail and the camera panel.

```bash
docker exec -it igvc_gazebo bash -c \
  "CAMERAS=front HEADLESS=1 RVIZ=1 NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

If that is still too slow, drop the camera resolution, which is the single
biggest lever:

```bash
docker exec -it igvc_gazebo bash -c \
  "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && \
   ros2 launch igvc_test_bringup gazebo_nav_test.launch.py \
     headless:=true rviz:=true sim_cameras:=front \
     sim_camera_width:=320 sim_camera_height:=180 sim_camera_hz:=5"
```

**Note on `CAMERAS=none`:** it is the fastest configuration, but
`bringup_smoke_test.sh` checks four camera topics as part of its 18 checks, so
a camera-less run **can never reach 18 of 18**. Keep the front camera if you
want the gate to pass.

### What you will and will not see

| | Tier C |
| --- | --- |
| The Gazebo 3D window | Technically opens, practically unusable. Do not bother |
| RViz, robot, TF, lidar returns, odometry trail | Yes, usable |
| RViz camera panel | Yes, but at a few frames per second |
| The robot driving the course under Nav2 | Yes, slower than real time |
| `render_check.sh` | Passes as tier C, exit 1 by design |
| `bringup_smoke_test.sh` | See the measured section below |
| Camera or timing numbers | **Do not report them** |

### Measured tier C numbers

Measured on this laptop with the GPU deliberately disabled through
`LIBGL_ALWAYS_SOFTWARE=1`, which is the same code path a machine with no usable
GPU takes. **These are laptop-with-GPU-disabled numbers, not numbers from any
real tier C machine, and certainly not AGX numbers.**

Method: 30 second windows, **counting messages** rather than asking for a
rate, with the real time factor computed from `/clock` rather than read off
Gazebo's own `real_time_factor`, which sits near 1.0 while cameras crawl.

| Configuration | Real time factor | Camera delivered | Lidar delivered |
| --- | --- | --- | --- |
| **640x360 @ 15 Hz, front camera** (the tier A default) | **0.608** | **1.63 Hz**, that is 11% of the 15 Hz asked for | 6.10 Hz |
| **320x180 @ 5 Hz, front camera** (the tier C recommendation) | **0.854** | **4.27 Hz**, that is 85% of the 5 Hz asked for | 8.53 Hz |
| 320x180, `sim_cameras:=none` | **0.996** | none by definition | 9.95 Hz |

**Read the camera column, not the resolution.** At the tier A default the
cameras deliver about a ninth of what they are asked for, and the whole
simulation drops to 0.61x real time carrying them. Ask for less and the
simulator very nearly keeps its promise: at 320x180 and 5 Hz it delivers 85%
of the requested frames at 0.85x real time. **That is the largest
configuration worth running on tier C**, and it is the one in the recipe
above.

Turning the cameras off entirely returns real time factor to 0.996, which
tells you plainly where the cost is: **the camera is the whole problem, and
the physics and lidar are fine.** That is also why navigation, odometry and
control work stay valid on tier C.


---

## 8. Drive it yourself

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
forward and let go of the key, the robot keeps going. Press `k`. Measured: it
travelled a further **3.672 m in the 4 s after `/cmd_vel` stopped**. Whether
that is a true absence of timeout or a long one is still an open question, and
`diff_drive_controller` on the real robot does time out, so the simulator is
the less safe of the two.

---

## 9. Check it actually works

`bootstrap.sh` runs the first two for you. These are the commands to re-run one
on its own.

```bash
docker exec -it igvc_gazebo bash -c \
  "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

Prints your adapter, your tier and what to do next. You do **not** need to
source ROS first; earlier versions of this guide said you did, and until
2026-09-17 you genuinely had to.

```bash
# the simulator and the topic contract: about 4 minutes on tier A
docker exec -it igvc_gazebo bash -c \
  "bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
```

Measured on tier A:

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

This script **never publishes a velocity command**. Measured on tier A:

```text
samples              : 893  at 9.5 Hz
distance travelled   : 83.0 m
centreline deviation : mean 0.67 m, max 1.86 m
yaw to lane          : mean 15.9 deg, max 48.0 deg
lane clearance       : worst +0.040 m (chassis), +0.081 m (Nav2 footprint)
                       0.0% of samples over the line
AUTONOMY CHECK: PASS
```

**`IN LANE` is provisional and you should read it as a number, not a verdict.**
It compares centreline deviation against a flat 2.00 m tolerance while the lane
width varies by a factor of two along the course, and it ignores the robot's
yaw. Three runs at honest sampling gave 2.04 (FAIL), 1.95 (PASS), 1.98 (PASS),
while every one of those runs drove 47 to 51 m under its own control with
**0.0%** of samples over the paint. If you see a FAIL there, look at the
clearance line before believing the robot misbehaved. Nobody is tuning the
tolerance until the metric is right.

---

## 10. When it goes wrong

**"There is no window."** You started it from PowerShell. See section 3.

**`docker: command not found` in the WSL2 shell, but it works in PowerShell.**
WSL Integration is off for your distro. Section 2.3, step 5.

**`RESULT: SOFTWARE FALLBACK` or `YOUR TIER: C` when you expected A.** Check
WSL Integration is on, then that `/usr/lib/wsl` exists inside the container:
`docker exec igvc_gazebo ls /usr/lib/wsl/lib`. Then check your NVIDIA driver is
current on **Windows**, not inside WSL. Do **not** install Linux NVIDIA drivers
inside WSL. And do not use `nvidia-smi` to diagnose this: it reports CUDA
working fine while OpenGL is on a software rasteriser.

**`bad interpreter: /usr/bin/env bash^M` or `cannot execute: required file not
found`.** Your checkout has Windows line endings. Section 2.1:

```bash
git config --global core.autocrlf input
cd /mnt/c/IGVC_robot_2027
git rm --cached -r . && git reset --hard
```

**`colcon build` fails on a package you never touched, usually
`zed_description`.** Your submodules are empty:
`git submodule update --init --recursive`.

**`docker pull` fails with `unauthorized` or `denied`.** The GHCR package is
not public yet. Tell Liam; it is a one-click fix on his side. Meanwhile:
`BUILD_IMAGE=1 bash scripts/gazebo/bootstrap.sh`.

**`docker pull` fails with a rate limit.** That is Docker Hub, not GHCR, and it
means something is pulling from Hub. The Gazebo image comes from GHCR, which
does not rate-limit anonymous pulls the way Hub does. Hub's limit is counted
per source IP, so everyone behind the room's NAT shares one allowance.

**"A simulator is ALREADY RUNNING in this container."** Exactly what it says.
Reset: `docker compose -f docker-compose.windows.yml restart igvc_gazebo`.

**The robot does not move with `NAV=1`.** Give it 60 seconds; Nav2 brings up
ten lifecycle nodes in sequence. If it still does not, look for `Failed to
bring up all requested nodes`, which is Nav2 losing a race against Gazebo's
startup on a loaded machine. Raise the delay:

```bash
ros2 launch igvc_test_bringup gazebo_nav_test.launch.py nav2_delay:=25.0
```

**Everything is slow.** Check nothing else is running
(`docker exec igvc_gazebo ps -ef | grep gz`), use `CAMERAS=front`, and check
your tier. Note that `real_time_factor` reads near 1.0 even when cameras crawl,
because gz-sim runs physics and rendering on separate threads. Count messages,
never trust RTF.

**The container exits immediately after `up -d`.** Check
`docker logs igvc_gazebo`. Usually Docker Desktop is out of memory or the WSL2
VM is out of disk.

**`no matching manifest`.** You are on an ARM machine (Snapdragon laptop, Apple
silicon). This image is x86-64 only. See section 5.

**Everything is correct and it still does not work.** Capture these four and
post them:

```bash
wsl -l -v                                        # from PowerShell
docker version | head -20
docker exec igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh" | tail -20
docker exec igvc_gazebo ps -ef | grep -cE "gz sim|rviz2"
```

---

## 11. What this does and does not do

**It does:** run the real 2026 navigation stack, that is the ground-truth lane
grid, the IGVC navigator, Nav2's controller, velocity smoother and collision
monitor, against Gazebo, with none of those nodes modified. The robot drives
the course, stays between the lines and goes around the barrels.

**It does not perceive anything.** This is the important one and it is easy to
misreport. The lane lines and the barrel positions come out of
`IGVC_track_generator/track_points.json`, not out of the camera or the lidar.
More precisely, `gazebo_nav_test_nav2_overrides.yaml` replaces the costmap
`plugins` list with `["lane_layer", "inflation_layer"]`, so Nav2's
`obstacle_layer` is **never instantiated**. Every obstacle the robot has ever
avoided in this simulator came from the JSON.

**The Hough lane detector does run** and needs nothing extra: CLAHE, Canny and
`HoughLinesP`, no torch. It produced 253 occupied cells in `/lane_map` over a
90 s self-driven run. Nothing plans on its output. **YOLOPv2
(`lane_segmentation_node`) does not run here**: it needs torch, which this
image does not have, and weights that are not in the repo.

**The camera point cloud is rotated 90 degrees.** Measured, not suspected, and
deliberately not fixed, because fixing it connects a Nav2 consumer that has
never been connected.

**`/fix` is not published** and **ros2_control is not in the loop**: Nav2's
twist goes straight to Gazebo's DiffDrive, so joint limits and the controller
update rate are untested in simulation.

**It is not a ZED camera.** The simulated cameras publish on the same topics
with the same frame ids and intrinsics as the real ZED wrapper, so downstream
code cannot tell the difference. The *content* is a plain pinhole camera with
exact ray-cast depth. The simulated field of view is 100 degrees and **which
lens is actually fitted to the robot is unknown**, so the sim is either 4.6
degrees too narrow or 26 degrees too wide and the sign is not known.

**Tier A is the only measured tier.** Tier B is unverified anywhere, tier C is
measured only on this laptop with its GPU switched off, and macOS is untested.
Nothing here has been run on the RTX 5080 laptop, the Windows 10 machine or
native Linux.

---

## Quick reference

```bash
# PowerShell, as Administrator, on a blank machine
winget install --id Git.Git --exact --source winget --accept-source-agreements
git config --global core.autocrlf input
wsl --install                 # THEN REBOOT
wsl --update
winget install --id Docker.DockerDesktop --exact --source winget   # THEN REBOOT
# then: Docker Desktop > Settings > Resources > WSL Integration > your distro > Apply & Restart

# WSL2 Ubuntu shell, from here on
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
bash scripts/gazebo/bootstrap.sh                # zero to verified, prints your tier
BUILD_IMAGE=1 bash scripts/gazebo/bootstrap.sh  # fallback if GHCR is unreachable

docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker compose -f docker-compose.windows.yml restart igvc_gazebo   # reset
docker compose -f docker-compose.windows.yml down                  # stop

# tier A or B
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
# tier C
docker exec -it igvc_gazebo bash -c "CAMERAS=front HEADLESS=1 RVIZ=1 NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
# a shell, for teleop
docker exec -it igvc_gazebo bash

# the three checks
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
| `/lane_ground_truth`, `/lane_map`, `/lane_costmap` | the ground-truth lane grids |
| `/sim/odom_body_frame` | Gazebo's RAW odometry. Do not use it; see `GAZEBO_SETUP.md` 9.3 |
