# Running the simulator on a Mac

Start at [`SETUP.md`](../../SETUP.md) if you have not picked a path yet.

> **No Mac has run this yet.** Everything here was built and verified without
> one, and section 6 says exactly what that means. **If you have a Mac, please
> work through [`MACOS_CHECKLIST.md`](MACOS_CHECKLIST.md) and hand it back.**
> It takes about half an hour and it is the only way this project gets real
> macOS data.

---

## 1. Which Macs, and what you get

**Apple Silicon only**: any M-series Mac, or the A18 MacBook. **Intel Macs are
not supported**; macOS 26 is their last release and nobody on the team has
one.

**Memory.** The whole stack, Gazebo plus Nav2's ten nodes plus the navigator,
peaked at **3.5 GB (3506 MB)** of resident memory in the Linux dry run (section 6).
A 16 GB Mac has room to spare. An **8 GB** Mac (the base MacBook) should run
the headless checks, but close other apps and expect the Gazebo window plus
RViz to be tight.

**The route is native: pixi and RoboStack, no Docker.** RoboStack packages ROS
2 Jazzy, Gazebo Harmonic and Nav2 as conda packages built for Apple Silicon,
and `pixi` installs them into `.pixi/` inside the repository. What that buys
you over Docker, which is why it is the Mac route:

| | Native (this guide) | Docker (the fallback, section 7) |
| --- | --- | --- |
| Runs as | Apple Silicon code | amd64 code under emulation |
| Rendering | **the Mac's GPU, through Metal** | the CPU only; Docker on macOS has no GPU |
| Gazebo window | **yes** | no, and there is no workaround |
| Verified | on the same recipes on Linux; not on a Mac | on Windows; not on a Mac |

Two things verified without a Mac that make the GPU claim more than hope: the
environment **resolves completely for Apple Silicon** (969 packages, every one
present), and conda-forge's macOS build of the renderer, `ogre-next` 2.3.3,
**ships `RenderSystem_Metal`** (read from the package's file list). Whether it
renders the camera and lidar correctly on a real Mac is the first thing the
checklist asks.

**Honest caveats, all from upstream rather than guesses:** ROS 2 Jazzy on macOS
is a **Tier 3** platform, Gazebo calls its macOS GUI "currently known to be
unstable", and RoboStack's Gazebo is **gz-sim 8.10.0** where the Docker image
has 8.15.0 (same Harmonic series). If the Gazebo window misbehaves, section 5
has a headless-plus-RViz way to work that avoids it.

---

## 2. Install, once

1. **Command line tools**, for `git`:

   ```bash
   xcode-select --install
   ```

2. **pixi**, then **open a new Terminal** so it is on your PATH:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

3. **Terminal must not be running under Rosetta.** Finder, Applications,
   Utilities, Terminal, Get Info: "Open using Rosetta" must be **off**. Check:

   ```bash
   uname -m        # must print arm64
   ```

That is all. No Docker, no XQuartz, no Homebrew ROS.

---

## 3. The commands

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
export ROS_DOMAIN_ID=<your number>     # see SETUP.md section 3
pixi run bootstrap
```

The first `pixi run` installs the environment from `pixi.lock`: about **6 GB**,
a few minutes on a good connection. Then `bootstrap` checks the machine,
populates submodules, builds the four simulator packages, runs
`render_check.sh` and the smoke test, printing PASS or FAIL for each step. Every
later `pixi run` starts in seconds.

**Daily use**, always from the repository root:

```bash
pixi run sim            # the simulator, with a Gazebo window and RViz
pixi shell              # a shell with ROS 2 on PATH, for anything else
pixi run autonomy       # the robot drives the course by itself
pixi run smoke          # the topic contract, and does it drive
pixi run render-check   # is the GPU rendering
```

To drive by keyboard, `pixi run sim` in one Terminal, then in a second:

```bash
pixi shell
source .ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**macOS may ask whether Python may accept incoming network connections.**
Allow it: that is ROS 2's DDS discovery, and without it nodes cannot see each
other. (Expected from how DDS works; not yet observed on a Mac.)

---

## 4. Expected output

Measured in the Linux dry run, which uses the same pixi environment and the
same commands on a clean Ubuntu container **with software rendering**. On a
Mac, expect **tier M** instead of tier C and faster cameras; the shapes are the
same.

| Command | Expect | Dry-run measurement |
| --- | --- | --- |
| `pixi install` (first run) | `The default environment has been installed.` | 118 s, 5.8 GB |
| build (inside `bootstrap`) | `PASS  four packages built` | 4 s |
| `render_check.sh` | `YOUR TIER: M` and `HARDWARE RENDERING (Apple GPU, Metal)` on a Mac. `YOUR TIER: C` in the dry run | camera and gpu_lidar both `PUBLISHING` |
| smoke test | `topic checks passed: 18   failed: 0`, `DRIVE TEST : PASS`, `FRAME TEST : PASS` | 18 of 18, heading 0.13 deg, ratio 0.9970; whole bootstrap 161 s |
| `pixi run autonomy` | `AUTONOMY CHECK: PASS` | PASS, 89.2 m over 100.0 s of sim time, 378 s of wall time at 0.29x real time |

**Nothing in the checks depends on how fast your Mac is.** Until 2026-09-24
they waited fixed numbers of seconds, which is the classic way a slow machine
fails a good simulator. They now wait for readiness, drive on simulation time
and watch simulation time, so a slow run takes longer rather than failing. In
the dry run the simulator ran at **0.29x real time** and the smoke test's drive
still covered 2.761 m, against 2.759 m on the fastest machine this project
has.

**`render_check.sh` exit codes:** 0 hardware (tiers A, B, M), 1 software
(tier C), 2 could not tell. **If you get `YOUR TIER: unknown` on a Mac, that is
the most useful thing you can report**: tier M's detection was written without
a Mac to read the log from, so copy the lines it printed under "GL / device
lines" into the checklist.

---

## 5. If something fails

**Record it in [`MACOS_CHECKLIST.md`](MACOS_CHECKLIST.md)** as well as fixing
it locally. That file is how the next Mac owner avoids the same problem.

| Symptom | Likely cause, and what to do |
| --- | --- |
| `ERROR: no ROS 2 at /opt/ros/jazzy, so this is not the Docker container` | You ran a script with `bash scripts/...`. Use `pixi run <task>` from the repo root |
| `FAIL  this shell is x86_64 on macOS` | Terminal is running under Rosetta. Section 2 step 3 |
| `pixi install` fails to solve | Your `pixi.lock` is out of date with `pixi.toml`, or the channel was unreachable. `git pull`, then retry |
| `YOUR TIER: unknown` | Section 4. Record the device lines |
| The Gazebo window crashes or draws nothing | The upstream-unstable part. Work headless with RViz instead, below |
| Topics FAIL in the smoke test while the log shows no error | The simulation clock may have stalled; the drive probe says so explicitly. Rerun once. If it repeats, record it |
| Nodes cannot see each other | The macOS firewall prompt was denied. System Settings, Network, Firewall |

**Headless with RViz**, which avoids the Gazebo GUI entirely and still shows
the robot, the lidar and the camera:

```bash
pixi shell
source .ws/install/setup.bash
ros2 launch igvc_test_bringup gazebo_sim.launch.py headless:=true rviz:=true
```

---

## 6. What was verified without a Mac, and what was not

**Verified 2026-09-24, in a clean Ubuntu 24.04 container with no ROS
installed**, using `scripts/gazebo/native_dryrun.sh`, which anyone can rerun:

- the environment resolves for **osx-arm64** and linux-64 from one `pixi.toml`
- `pixi install --locked` succeeds from the committed lock
- the repository at an ordinary home-directory path, with nothing from the
  Docker layout, builds and passes `render_check.sh`, the smoke test and the
  autonomy check under `pixi run`
- running a script outside pixi fails with a message saying what to do
- conda-forge's macOS `ogre-next` carries the Metal render system
- peak memory of the whole stack: **3.5 GB (3506 MB)**

**Not verifiable without a Mac, and asked for on the checklist:**

1. **Metal rendering of the camera and lidar**, and the tier M detection.
2. **The Gazebo window.** The launch file starts the server and GUI as
   separate processes on macOS, because the `gz` CLI there refuses to run both
   in one; that branch has never executed.
3. **Headless sensor rendering without `--headless-rendering`**, a flag that
   selects EGL and exists only on Linux. The macOS branch leaves it off.
4. **The macOS userland**: BSD `ps`, `pgrep` and `pkill`, and the firewall
   prompt. The scripts were written for both; only Linux has run them.
5. **Every timing** on Apple hardware.

---

## 7. The Docker fallback: headless, if the native route fails

Docker Desktop on macOS gives containers **no GPU**, so this path is software
rendering only with **no window**, and on Apple Silicon the image runs under
emulation because it is published for amd64 only. It works for the headless
checks and for navigation work that does not need the camera to be fast.

Install Docker Desktop for Mac and give it at least 8 GB of memory (Settings,
Resources). **Settings, General, Virtual Machine Manager must be "Apple
Virtualization framework"**, which has been the default since Docker Desktop
4.44 and is what lets it use **Rosetta**, on by default, for amd64 images. The
alternative "Docker VMM" has no Rosetta and emulates much more slowly.

```bash
docker pull --platform linux/amd64 ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
docker tag ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest igvc-gazebo-jazzy:latest

COMPOSE_FILE=docker-compose.mac.yml SERVICE=igvc_gazebo_mac \
CONTAINER=igvc_gazebo_mac PULL_PLATFORM=linux/amd64 \
bash scripts/gazebo/bootstrap.sh
```

**`--platform linux/amd64` is required, not optional.** Without it Apple
Silicon asks the registry for arm64 and gets `no matching manifest for
linux/arm64` (verified 2026-09-24 by making exactly that request). Two warnings
are expected and harmless on this path: `cwd is not under /mnt` (a Windows
check) and `TIER C`.

Then the checks, all headless. **Do not pass `RVIZ=1`**, which is the one
switch that tries to open a window:

```bash
docker exec igvc_gazebo_mac bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh
docker exec igvc_gazebo_mac bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh
docker compose -f docker-compose.mac.yml down        # when finished
```

Verified on Windows with the Mac file, which reproduces everything except the
processor: 18 of 18, autonomy PASS. **Why not XQuartz for a window?** Its GLX
is far below the OpenGL 3.3 Gazebo's renderer needs, so `gz sim` fails outright.

**Not built: a native arm64 image**, which would remove the emulation from
this fallback. Every package it needs exists for arm64, but the native route
above already runs natively and with a GPU, so it is not the priority. See
`docs/MAINTAINING_ENVIRONMENTS.md`.
