# Running the simulator on a Mac

> ## NO MAC HAS RUN THIS.
>
> Every instruction here was written by reading the code and verified on
> Windows, not on macOS. The parts that only a Mac can test are listed in
> section 6 and collected as a tear-off list in
> `MAC_VERIFICATION_CHECKLIST.md`. **If you have a Mac, please work through
> that checklist and hand it back filled in.** Nobody on this team has this
> data for any Mac.

---

## 1. What you get, and what you cannot have

**You get: the automated checks, headless.** `render_check.sh`,
`bringup_smoke_test.sh` and `autonomy_check.sh` all run without a display, and
the navigation work they exercise is real. You can develop and test navigation,
odometry, the topic contract and the control path.

**You cannot have a Gazebo or RViz window. This is not a configuration problem
and there is no workaround.** Two independent reasons:

1. **Docker Desktop on macOS gives containers no GPU.** Docker's own
   documentation states that GPU support in Docker Desktop is only available on
   Windows with the WSL2 backend. Containers run inside a Linux VM on Apple's
   Virtualization framework, which exposes no 3D-capable virtual GPU. There is
   no `--gpus` equivalent to try.
2. **XQuartz cannot display Gazebo.** Its indirect GLX is far below the
   **OpenGL 3.3** that gz-sim's `ogre2` render engine requires. Sources put
   XQuartz's ceiling at either 1.4 or 2.1 depending on which path is measured;
   both are below 3.3, so the conclusion is the same either way. `gz sim` fails
   with a render-engine error rather than running slowly.

**So do not set `DISPLAY`, do not install XQuartz for this, and do not mount
X11 sockets.** `docker-compose.mac.yml` deliberately contains no display
plumbing at all.

**On Apple Silicon there is a third cost: emulation.** The published image is
**amd64 only**, verified: `docker manifest inspect` returns one `amd64/linux`
manifest plus a buildx attestation entry, and **no arm64 entry**. So an
M-series Mac runs it translated. See section 7.

**What tier are you?** Tier C, software rendering, in the project's own
shorthand. That is a supported configuration: measured on the reference
Windows laptop with its GPU switched off, tier C passes
`bringup_smoke_test.sh` at **18 of 18** with odometry matching the simulator's
ground truth to **0.18 percent**. What tier C loses is camera throughput, so
**do not take camera or timing measurements on a Mac and do not compare them
with anyone else's.**

---

## 2. Install

1. **Docker Desktop for Mac.** `https://www.docker.com/products/docker-desktop`.
   Take the Apple Silicon build on an M-series machine and the Intel build on
   an Intel Mac; that is about the Docker app itself, not about our image.
2. **Give Docker Desktop enough memory.** Settings, Resources. **8 GB
   minimum, 12 GB comfortable.** The default is often too small once Gazebo
   and ten Nav2 nodes are running, and the symptom is a container that dies
   without a message.
3. **Git.** Preinstalled on macOS, or `xcode-select --install`.
4. **Do not enable "Use Rosetta for x86_64/amd64 emulation" expecting it to
   help.** It is off by default and it is not required. If you do try it, note
   that Rosetta cannot advertise AVX or AVX2 to Linux guests, and Mesa's
   software rasteriser is exactly the kind of code that benefits from those,
   so it may be slower rather than faster. **Record which setting you used on
   the checklist**, because nobody knows which is better here.

---

## 3. The commands, in order

Everything is a normal macOS Terminal. There is no WSL2 and no equivalent, so
unlike the Windows guide there is no "wrong shell" trap.

**Step 1, clone.** The `--recurse-submodules` is not optional: `zed_description`
is built by name, so the workspace build fails without it, and
`IGVC_track_generator` holds the track data every navigation node reads.

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
```

Expect nine submodules:

```bash
git submodule status | wc -l      # want 9
git submodule status | grep '^-'  # want no output
```

**Step 2, pull the image, explicitly as amd64.**

```bash
docker pull --platform linux/amd64 ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
docker tag ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest igvc-gazebo-jazzy:latest
```

About 1.2 GB over the wire, unpacking to about 5.6 GB. The package is public,
so no `docker login`. The `--platform` flag is what stops Docker warning about
a platform mismatch on Apple Silicon.

**Step 3, everything else in one command.** `bootstrap.sh` takes the macOS
settings as environment variables, so there is no separate Mac script to drift
out of date:

```bash
COMPOSE_FILE=docker-compose.mac.yml \
SERVICE=igvc_gazebo_mac \
CONTAINER=igvc_gazebo_mac \
PULL_PLATFORM=linux/amd64 \
ROS_DOMAIN_ID=<your number> \
bash scripts/gazebo/bootstrap.sh
```

**Two warnings you will see, and both are expected on a Mac:**

- `WARN  cwd is not under /mnt, so this may not be a WSL2 shell.` That check
  exists for Windows users who opened the wrong terminal. On a Mac there is no
  `/mnt` and no WSL2, so it always fires. Ignore it.
- `WARN  TIER C, SOFTWARE RENDERING. The GPU is not reaching the container.`
  That is correct and expected. `render_check.sh` exits 1 on tier C by design,
  and `bootstrap.sh` treats it as a warning rather than a failure, so the run
  continues.

**If you would rather do it by hand:**

```bash
docker compose -f docker-compose.mac.yml up -d igvc_gazebo_mac

docker exec igvc_gazebo_mac bash -c '
  source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws &&
  colcon build --symlink-install \
    --base-paths /root/ros2_ws/src/IGVC_robot_2026/src \
    --packages-select zed_description igvc_test_description igvc_test_bringup igvc_lane_detection'
```

**Step 4, the checks.** All three are headless by default. **Do not pass
`RVIZ=1`**: it is the one switch that tries to open a window, and on a Mac it
will fail.

```bash
docker exec -it igvc_gazebo_mac bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
docker exec -it igvc_gazebo_mac bash -c "bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
docker exec -it igvc_gazebo_mac bash -c "bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
```

**Step 5, stop it when you are done.**

```bash
docker compose -f docker-compose.mac.yml down
```

---

## 4. Expected output

Measured by running `docker-compose.mac.yml` on Windows in an isolated
project, which reproduces the Mac condition in every respect except processor
architecture: no GPU request, no display mounts, software rendering.
**Timings on a Mac will be worse, and on Apple Silicon possibly much worse,
because of emulation. Treat the shapes as expected and the numbers as a
ceiling.**

| Command | Expect | Windows-side measurement |
| --- | --- | --- |
| `docker compose up -d` | container `igvc_gazebo_mac` running | under 1 s |
| `colcon build` | `build ok`, exit 0 | 13 s natively. **This is the step most likely to be slow or to fail under emulation** |
| `render_check.sh` | `ADAPTER: llvmpipe`, `RESULT: SOFTWARE RENDERING`, `YOUR TIER: C`, **exit 1 by design** | 14 s |
| `bringup_smoke_test.sh` | `topic checks passed: 18   failed: 0`, `DRIVE TEST : PASS`, `FRAME TEST : PASS` | see the report |
| `autonomy_check.sh` | `AUTONOMY CHECK: PASS`, though `IN LANE` is a known coin flip | see the report |

**`render_check.sh` exiting 1 is correct on a Mac** and means "software
rendering", not "broken". The exit codes are 0 hardware, 1 software, 2 could
not tell.

---

## 5. If something fails

**Record it on `MAC_VERIFICATION_CHECKLIST.md` and hand that back**, rather
than only fixing it locally. That file is the only way this project gets any
macOS data at all.

A few that are predictable:

| Symptom | Likely cause |
| --- | --- |
| `no matching manifest for linux/arm64` | You left off `--platform linux/amd64`. Section 3 step 2 |
| The container dies with no message | Docker Desktop memory. Give it 8 GB or more |
| `colcon build` hangs or takes very long | Emulation. Expected, and the number is what we want. Record it |
| A Qt error about connecting to a display | Something started RViz. Do not pass `RVIZ=1`. If it is not RViz, retry that one command with `docker exec -e QT_QPA_PLATFORM=offscreen ...` |
| `gz sim` fails with a render-engine or OpenGL error | You are trying to open a window. Section 1. There is no fix |
| `colcon build` fails on `zed_description` | Submodules are empty. `git submodule update --init --recursive` |

---

## 6. What could not be tested without a Mac

Stated plainly so nobody mistakes the Windows-side verification for a macOS
result:

1. **Whether the colcon build completes under Rosetta or QEMU emulation, and
   how long it takes. This is the single most likely thing to fail** and it is
   first on the checklist. Compiling C++ under instruction translation is the
   heaviest thing in the whole sequence.
2. **Whether `docker pull --platform linux/amd64` behaves as expected** on
   Apple Silicon, and how long it takes.
3. **macOS Docker Desktop's own behaviour**: its VM, its default resource
   limits, and whether the defaults are enough.
4. **Whether Rosetta or the default QEMU path is faster** for this workload.
5. **Any timing at all.** Tier C on a Mac is a guess until somebody measures
   it, which is why the checklist asks for timings on every step.

---

## 7. A native arm64 image: scoped, not built

**Not started, and deliberately so.** Building a multi-architecture image under
emulation takes hours and can fill a disk. This is a written estimate only.

**What it would fix.** It removes the emulation layer on Apple Silicon. The
build would run natively, which is where most of the pain is. **It does not
give you a window**: rendering stays on the CPU, because the GPU limitation in
section 1 is about Docker on macOS and not about architecture. So arm64 turns
"emulated software rendering" into "native software rendering", which is a
real improvement to build times and nothing else.

**Why it is feasible**, verified from the actual package indexes rather than
from documentation:

- REP-2000 lists Ubuntu Noble **arm64 as a Tier 1 platform for ROS 2 Jazzy**.
- `packages.ros.org/ros2/ubuntu/dists/noble/Release` declares
  `Architectures: i386 amd64 arm64 armhf`.
- The arm64 package index carries **3667 `ros-jazzy-*` packages**, including
  every one this image installs: `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-sim`,
  `ros-jazzy-ros-gz-bridge` and the rest.
- **This image needs no ZED SDK**, which is the usual arm64 blocker on this
  project. The Dockerfile is a plain apt install on top of
  `ros:jazzy-ros-base`, which is itself multi-arch.

**What it would take:**

1. A `docker buildx` multi-architecture build, `linux/amd64,linux/arm64`.
2. Somewhere to push both manifests under one tag. GHCR already does this; the
   existing package would gain an arm64 entry alongside the current amd64 one,
   so the image reference in every document stays the same.
3. **Native arm64 build hardware, or patience.** Building arm64 under QEMU on
   an x86 machine is the hours-long path. A GitHub Actions arm64 runner or an
   actual Mac would do it properly.
4. One round of verification on a real Mac afterwards.

**Estimate: half a day**, most of it the buildx pipeline and the verification
round, assuming native arm64 build hardware. Materially longer if it has to be
cross-built under emulation.

**Do not start this before somebody has returned a filled-in
`MAC_VERIFICATION_CHECKLIST.md`.** If emulation turns out to be tolerable, this
is a nice-to-have. If the emulated build fails outright, this becomes the only
way a Mac participates, and that changes its priority.
