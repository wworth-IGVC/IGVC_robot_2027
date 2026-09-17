# Gazebo setup: which version, how it is installed, and proof it uses the GPU

**Created:** 2026-09-10. **Ported from Humble to Jazzy:** 2026-09-15.
**Machine under test:** RTX 5070 Ti Laptop GPU, 12227 MiB, driver 610.88,
Intel Core Ultra 9 275HX, Windows 11, Docker Desktop 29.7.2 (WSL2 backend)

This closes **RQ-01** (which Gazebo, and is it viable) and **RQ-02** (does
Gazebo render on the GPU) with measurements rather than sources. Both were P0
and both blocked everything else.

Section 2 was rewritten on 2026-09-15 when the distro changed from Humble to
Jazzy. **The GPU findings in sections 3, 3A and 4 were not affected** and were
re-verified rather than re-derived: they are properties of WSL2, Docker and the
GPU, with nothing ROS-version-specific in them.

---

## 1. Short version

| Question | Answer |
| --- | --- |
| Which distro | **Jazzy**, supported to May 2029. Humble ends May 2027, one month before the competition |
| Which Gazebo | **Harmonic**, `gz-sim` 8 — and on Jazzy this is REP 2000's Tier 1 pairing, not an off-label choice |
| How | Docker image `igvc-gazebo-jazzy:latest`, built from `docker/Dockerfile.gazebo-jazzy` |
| Source build needed | **No — and now not even for `gz_ros2_control`**, which was the one exception on Humble |
| Third-party apt repo | **None.** Everything comes from packages.ros.org |
| GPU rendering | **Yes**, `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, but only with four specific settings |
| Three cameras at 30 Hz | **No.** At 1280x720/30 Hz you get ~7 Hz each |
| Three cameras, workable | **Yes at 640x360/15 Hz**: ~11 Hz each, physics at real time |
| A robot that drives | **Yes.** IGVC course world plus a spawn/bridge launch file. `bringup_smoke_test.sh`: 18 of 18 checks, 0 failures. See section 8 |
| Downstream nodes | **Yes.** The bridge publishes the section 3.7 contract names; the real 2026 ground-truth, navigator and Nav2 stack run against it. Section 9 |
| A robot that drives ITSELF | **Yes.** 122.9 m of autonomous driving, max 1.79 m off the lane centreline, no `/cmd_vel` from the test. Section 10 |
| Perception | **No, and do not imply otherwise.** Lanes and barrels come from `track_points.json`; nothing plans on the camera or lidar. Section 10.3 |

**New here? Read `GAZEBO_QUICKSTART.md` instead**, or the Word version of it.
This file is the reference and explains why; that one gets you to a driving
robot in about 45 minutes without assuming you have read this.

Run it **from a WSL2 shell, not PowerShell** (see section 3A):

```bash
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
docker compose -f docker-compose.windows.yml build igvc_gazebo
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
```

**`up -d`, not `run --rm`.** `run` creates a new container with a random name
every time, so a second `run` gives you a second simulator rather than a second
shell into the first. Two simulators on one ROS domain both publish `/clock`,
`/odom` and `/tf` for a model named `igvc_robot`, and the navigation stack then
plans against a robot whose pose jumps between them - which reads as a
navigation bug and is not one. That cost real time on 2026-09-15; see 10.5.

Then, before trusting anything:

```bash
docker exec -it igvc_gazebo bash -c   "source /opt/ros/jazzy/setup.bash && bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

And to see the robot drive the course by itself:

```bash
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

---

## 2. RQ-01: Jazzy + Harmonic, and why the distro moved

**Decision: ROS 2 Jazzy + Gazebo Harmonic.**

This changed on 2026-09-15. The original decision was Humble + Harmonic, and
the reasoning that produced it is kept in section 2.5 for provenance, because
it was right about the simulator and wrong about the layer underneath.

### 2.1 Why Jazzy

Verified against the REP 2000 source on 2026-09-15:

```text
Humble Hawksbill (May 2022 - May 2027)
Jazzy Jalisco   (May 2024 - May 2029)
```

The competition is **4 to 8 June 2027**. Humble reaches end of life the month
before it.

This is the same argument that ruled out Gazebo Fortress, which also ends in
May 2027. The original analysis applied it to the simulator and never turned it
on the distro underneath. Jazzy also covers the 2028 team, which matters for a
group with annual turnover.

Harmonic does not change. What changes is that REP 2000 lists Fortress as
Humble's pairing and **Harmonic as Jazzy's**, so the combination goes from
deliberate-but-off-label to the supported one. Every measurement already taken
stays valid and stops swimming upstream.

### 2.2 Jazzy is where the code already was — but not the robot images

This split matters, and getting it wrong in either direction is easy.

**Already Jazzy — the code and the development environment:**

- The 2026 competition branch `more_diverging_changes` is **Jazzy code**,
  proven by it failing to compile on Humble (`rclcpp::Clock::now()` is const in
  Jazzy and not in Humble).
- A mesh URI left in the repo points at
  `.../DevEnv/jazzy_ws/IGVC_robot_2026/...`, so the competition robot ran out
  of a `jazzy_ws`.
- The org's `dev_env` image is Jazzy, and so was the private `dev-zed`:
  `docker-compose.yml`'s `igvc_dev_zed` service sources
  `/opt/ros/jazzy/setup.bash`.
- The 2027 team independently standardised on `osrf/ros:jazzy-desktop` and
  `jazzy_ws` in DevEnv, and built `igvc_test_bringup`, `igvc_test_description`,
  `igvc_simulation_interface` and `ping_location` under it.

**Still Humble — the robot-side images.** Per the audit in section 5.4 of
`DOCKER_CHANGES.md`, done by reading the Dockerfiles rather than the labels:
`jetson-zed`, `jetson-ros-base`, `jetson-isaac-ros` and `isaac-ros` are all
Humble.

Two traps in that area, both already documented and both easy to fall into:

- `docker-compose.jetson.yml` describes `igvc_jetson_zed` as "ROS 2 Jazzy + ZED
  SDK 5". **That comment is stale.** The image is Humble and the service's own
  commented-out command sources `/opt/ros/humble/setup.bash`.
- `jetson-ros-base` is tagged `jazzy-36.4.7-2` but its Dockerfile builds
  **Humble**. The tag is actively misleading.

So this is a real migration on the robot side, not a formality. The Jetson
images will eventually need the same treatment, and nothing here has done that
yet. **The EOL date decides it regardless:** Humble is unsupported before the
team ever competes on it, so the robot-side images need to move whether or not
the simulator leads the way.

The Humble images in this repo came from forking `main`, a pre-competition
snapshot, and from matching `jetson-ros-base` on the Jetson drive stack.

### 2.3 What moving to Jazzy deletes

Everything comes from **packages.ros.org**. Verified against the live noble
index on 2026-09-15:

```text
ros-jazzy-ros-gz                 1.0.24-1noble
ros-jazzy-ros-gz-sim             1.0.24-1noble
ros-jazzy-ros-gz-bridge          1.0.24-1noble
ros-jazzy-ros-gz-image           1.0.24-1noble
ros-jazzy-ros-gz-interfaces      1.0.24-1noble
ros-jazzy-simulation-interfaces  1.5.1-1noble
```

Gazebo itself arrives through the `ros-jazzy-gz-*-vendor` dependency chain in
the same index. Consequences:

- **No osrfoundation apt repo.** The Dockerfile no longer fetches a
  third-party GPG key or writes a source list. That is roughly 15 lines gone.
- **No flavour split and therefore no conflict.** `ros-jazzy-ros-gzharmonic`
  and `ros-jazzy-ros-gzfortress` **do not exist** — confirmed absent from the
  index. There is one `ros-jazzy-ros-gz` because there is nothing to
  disambiguate. The Debian `Conflicts:` problem documented for Humble in
  section 2.5 simply does not arise.

### 2.4 The one genuine source build is gone

This is the biggest practical gain and it reverses a documented blocker.

```text
ros-jazzy-gz-ros2-control  1.2.20-1noble    PRESENT
```

On Humble, `gz_ros2_control` had no Harmonic binary anywhere:
`ros-humble-gz-ros2-control` 0.7.20 was built against `libignition-gazebo6`,
which is Fortress despite the `gz-` name, so installing it would have dragged a
second Gazebo into the image. It was the single source build standing between
the team and Stage 2.

On Jazzy it depends on the same `ros-jazzy-gz-*-vendor` packages as `ros_gz`,
so it installs cleanly alongside. It **is** installed in the image.

**Installing it is not the same as adopting it.** RQ-06 — `gz_ros2_control`
versus a `grr_hardware/GazeboDriveHardware` topic bridge mirroring
`IsaacDriveHardware` — is still open and still a team decision. It is present
only so that trying the `ros2_control` path costs nothing. Do not treat its
presence as the decision having been made.

### 2.5 The superseded Humble reasoning, kept for provenance

The original section 2 answered "Fortress or Harmonic, on Humble". Its
conclusion about the simulator was right and its evidence still holds; it was
simply asking the question one layer too high up.

It recorded that the research artifact recommended Fortress because Humble plus
Harmonic supposedly required building `ros_gz` from source, and that this
premise was **false** — `ros-humble-ros-gzharmonic` 0.244.12-3jammy and its
five siblings shipped as prebuilt jammy debs from packages.osrfoundation.org,
checked against the live index on 2026-09-10. That removed the maintainability
argument for Fortress and left its May 2027 EOL unanswered.

It also recorded the real Debian `Conflicts:` between the Fortress and Harmonic
flavours on Humble:

```text
ros-humble-ros-gzharmonic         Conflicts: ros-humble-ros-gz, ros-humble-ros-gzgarden
ros-humble-ros-gzharmonic-bridge  Conflicts: ros-humble-ros-gz-bridge, ros-humble-ros-gzgarden-bridge
```

Both findings are now historical. The Humble image is preserved at
`docker/deprecated/Dockerfile.gazebo-harmonic`; see
`docker/deprecated/README.md`.

---

## 3. RQ-02: it renders on the GPU, and the default is a silent CPU fallback

This is the section to read before doing any camera work.

**First run, with `--gpus all` and nothing else:**

```text
GL_VERSION  = 4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.4
GL_VENDOR   = Mesa
GL_RENDERER = llvmpipe (LLVM 15.0.7, 256 bits)
```

`gz sim` started, the world loaded, and **both the camera and the gpu_lidar
topics published**. Everything looked fine. It was rendering on the CPU. This
is exactly the failure mode RQ-02 named: "software fallback that merely looks
slow is the dangerous failure mode, because it invalidates any camera-based
perception work." Nothing in Gazebo's own output says so; you have to read
`~/.gz/rendering/ogre2.log`.

**After the fix:**

```text
GL_VERSION  = 4.2 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.4
GL_VENDOR   = Microsoft Corporation
GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
```

### What the fix is, and why each part is load-bearing

WSL2 does not give Linux containers a native NVIDIA OpenGL driver. It gives a
GPU device (`/dev/dxg`) plus a Direct3D 12 shim, and Mesa's `d3d12` Gallium
driver renders GL on top of it. That is the mechanism the WSLg GPU fixes in
Gazebo Garden and Harmonic were written against, and it is another reason
Fortress was the wrong pick for an all-Windows team.

Four things are required. Miss any one and you silently get llvmpipe:

1. **`NVIDIA_DRIVER_CAPABILITIES=all`** (set in the Dockerfile). The container
   runtime default is `utility,compute`, which does not include graphics.
2. **`--device=/dev/dxg`**. The WSL GPU device.
3. **`-v /usr/lib/wsl:/usr/lib/wsl:ro` and `LD_LIBRARY_PATH=/usr/lib/wsl/lib`.**
   This is the part that is easy to miss. The image already contains Mesa's
   `d3d12_dri.so`, but it cannot load without `libd3d12core.so` and
   `libdxcore.so`, which live in `/usr/lib/wsl/lib` and are **not** in the
   container by default. Mesa's failure to load them is not an error; it is a
   fallback to llvmpipe.
4. **`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`**. This machine has two GPUs: the
   Intel Arc iGPU in the Core Ultra 9 275HX, and the RTX 5070 Ti. Without this,
   Mesa picks the **Intel** adapter and `gz sim` aborts inside Intel's own WSL
   driver:

   ```text
   Object ".../iigd_dch.inf_amd64_.../libigc.so"
   Object ".../libLLVM-14.so", in llvm::report_fatal_error(char const*, bool)
   Aborted (Signal sent by tkill())
   ```

   That crash looks like a Gazebo bug and is not one. Any team laptop with an
   Intel iGPU plus an NVIDIA discrete GPU will hit it.

All four are wired into the `igvc_gazebo` service in
`docker-compose.windows.yml`, with the same explanation inline so nobody
"cleans up" the mount.

---

## 4. Measured performance

Three runs, all on the machine above, all headless
(`gz sim -s -r --headless-rendering`). Worlds and scripts are in
`scripts/gazebo/`.

| Scene | Sensors | Achieved | Stability |
| --- | --- | --- | --- |
| `render_check.sdf` | 1 camera 640x480 @30 Hz, 1 gpu_lidar @10 Hz | **27 Hz**, 9 Hz | survived 70 s, physics RTF 1.00 |
| `three_camera_load.sdf` | 3 rgbd 1280x720 @30 Hz, gpu_lidar @10 Hz | **7 Hz** each, lidar 2 Hz | survived 60 s |
| same, 640x360 @15 Hz | 3 rgbd 640x360 @15 Hz, gpu_lidar @10 Hz | **11 Hz** each, lidar 7 Hz | survived 60 s, physics RTF 1.00 |

**Reading these honestly.** In gz-sim 8 physics and rendering run on separate
threads, so the `real_time_factor` the server reports can sit at 1.0 while the
cameras fall far behind their requested rate. That is what happens at 720p: the
server reports RTF ~1.0 and still delivers 7 Hz per camera instead of 30. **The
reported RTF is not sufficient evidence that the sensors are keeping up.** Count
messages, which is what `scripts/gazebo/sensor_bench.sh` does.

At 720p the sensors also take roughly 35 s to initialise before the first frame,
against roughly 15 s at 640x360.

**Working conclusion for acceptance criterion 6:** the Gazebo path is usable on
a 12 GB laptop, but not at 720p/30 Hz on three cameras. Budget **640x360 at
15 Hz** for three-camera work and raise it only if a measurement supports it.
Whether YOLOPv2 and the 13-class detector tolerate 640x360 is an open perception
question, not a simulator question, and it should be answered before the world
and shim work hardens around a resolution.

### One known defect

`gz sim` segfaults **on teardown** on this render path (exit 139, a stack trace
in a render worker thread). It does not affect a running server: continuous runs
survived 60 to 70 s with no interruption and correct data throughout. It shows
up when the server exits, so `--iterations N` runs always end in a core dump
even though the run itself completed. Treat a 139 at shutdown as cosmetic for
now; do not treat a crash **during** a run as the same thing.

---

## 3A. Seeing the simulation: run compose from WSL2, not PowerShell

**Decided and verified 2026-09-15.** This is the other half of RQ-02 and it has
a cheap answer.

Docker Desktop's own Linux VM has **no display**: `/tmp/.X11-unix` is empty and
there is no WSLg inside it. A container launched from PowerShell therefore can
never show a window, no matter how the GPU is configured. Headless works, and
that is all.

Launch the *same container* from a **WSL2 Ubuntu shell** instead and the
distro's WSLg sockets mount through. The Gazebo GUI opens as an ordinary
Windows window, on the same D3D12 NVIDIA render path:

```bash
# from a WSL2 Ubuntu shell
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
docker compose -f docker-compose.windows.yml run --rm igvc_gazebo
# then, inside the container
gz sim /root/ros2_ws/src/IGVC_robot_2026/scripts/gazebo/render_check.sdf
```

Measured from inside the container launched this way:

```text
DISPLAY=:0
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
OpenGL version string:  4.2 (Compatibility Profile) Mesa 23.2.1
```

**Why this is the right answer for this team.** It needs no second operating
system, no dual boot, no X server on Windows, and no change to the Docker setup
anyone already has working. The only prerequisite is a checkbox.

**One-time setup per machine:** Docker Desktop, Settings, Resources, WSL
Integration, enable the distro. The distro version does **not** matter, because
ROS and Gazebo live in the container; Ubuntu 26.04 works fine as a launcher even
though it has no ROS distro of its own - neither Humble nor Jazzy. This also
means the "WSL has no Humble" objection never applied to the GUI question, and
it does not come back as a "WSL has no Jazzy" objection either.

Stored in `%APPDATA%\Docker\settings-store.json` as:

```json
"EnableIntegrationWithDefaultWslDistro": true,
"IntegratedWslDistros": ["Ubuntu-26.04"]
```

The compose service passes `DISPLAY=${DISPLAY:-}`, so running it from
PowerShell is still safe: `DISPLAY` is simply empty and headless work is
unaffected. **Prefer WSL2 for everything** rather than remembering which shell
gives which capability.

Note the Windows-filesystem path (`/mnt/c/...`) is slower than a native WSL
path, but colcon output already lives in named volumes, so this costs little.

---

### Native Linux, where none of this applies

Everything in 3A is about WSL2. On native Linux there is a real GPU and a real
X or Wayland server, so OpenGL works the ordinary way. Use `igvc_gazebo_linux`
in `docker-compose.yml`:

```bash
xhost +local:docker
docker compose up -d igvc_gazebo_linux
```

It is **not yet verified on a Linux machine** - see `DOCKER_CHANGES.md` 13.4.
Run `render_check.sh` there too: a missing or mismatched NVIDIA driver still
falls back to software silently, it just does so for different reasons.

---

## 4A. Verification log

Two runs are recorded. The **2026-09-15 Jazzy** run is current. The
**2026-09-10 Humble** run is kept because it is the baseline the port is
measured against: it shows the port changed the distro without regressing
anything.

Everything below was run through the compose service, with the scripts executed
straight off the bind mount, never from hand-built `docker run` commands.

### Jazzy, 2026-09-15 — current

```text
docker compose -f docker-compose.windows.yml run --rm igvc_gazebo \
  bash -lc "... scripts/gazebo/render_check.sh"

  ROS_DISTRO      = jazzy
  ubuntu          = 24.04.4 LTS (Noble Numbat)
  gz sim versions = 8.15.0
  GL_VERSION      = 4.6 (Core Profile) Mesa 25.2.8-0ubuntu0.24.04.2
  GL_RENDERER     = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
  camera topic    : PUBLISHING
  gpu_lidar topic : PUBLISHING
  RESULT: HARDWARE RENDERING (NVIDIA)          exit 0
```

```text
  camera  -> sensor_msgs/Image       PASS
  lidar   -> sensor_msgs/LaserScan   PASS
  clock   -> rosgraph_msgs/Clock     PASS
  passed: 3   failed: 0               exit 0
```

`gz sim` reports **8.15.0**, the same Gazebo build the Humble image ran, so the
section 4 performance numbers carry over exactly rather than by analogy.

`gz_ros2_control` was confirmed as more than an apt entry:

```text
/opt/ros/jazzy/lib/libgz_ros2_control-system.so    present
```

**GUI from WSL2, same day, confirmed visually.** `gz sim -r render_check.sdf`
launched from a WSL2 shell opened as an ordinary Windows window: the
`render_check` world with its full entity tree (`ground_plane`, `target_box`,
`sensor_rig`, `sun`), dartsim physics, shadows being cast, and a real-time
factor of **99.77%**. Note the section 4 caveat — physics at real time says
nothing about sensor rates, which must be measured by counting messages.

### Humble, 2026-09-10 — superseded baseline

```text
  GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
  camera topic    : PUBLISHING
  gpu_lidar topic : PUBLISHING
  RESULT: HARDWARE RENDERING (NVIDIA)

  camera  -> sensor_msgs/Image       PASS
  lidar   -> sensor_msgs/LaserScan   PASS
  clock   -> rosgraph_msgs/Clock     PASS
  passed: 3   failed: 0
```

### What this proves, and what it does not

The whole chain is proven on Jazzy: Gazebo Harmonic renders on the discrete
GPU, the GUI reaches the Windows desktop through WSLg, and `ros_gz` delivers
correctly typed sensor data and simulated time to ROS 2 nodes. That is the part
of the section 3.7 interface contract Gazebo can satisfy without any of our own
code.

Not covered: the ZED namespace, the joint-state pair and TF, which depend on
RQ-03 and RQ-06. Also untested on any machine other than the one named at the
top — the RTX 5080 Laptop and the Windows 10 machine remain unverified, and the
Intel-adapter crash in section 3 depends on which GPUs a laptop has.

---

## 5. What is in the image

Base `ros:jazzy-ros-base` (Ubuntu 24.04 noble), plus, **all from
packages.ros.org** — no third-party apt repo and no GPG key to fetch:

- `ros-jazzy-ros-gz` 1.0.24 — the metapackage: `ros_gz_sim`, `ros_gz_bridge`,
  `ros_gz_image`, `ros_gz_interfaces`. Gazebo Harmonic itself arrives through
  its `ros-jazzy-gz-*-vendor` dependency chain.
- `ros-jazzy-gz-ros2-control` 1.2.20 — installs
  `/opt/ros/jazzy/lib/libgz_ros2_control-system.so`, verified present. This had
  no Harmonic binary at all on Humble. **Available, not adopted:** RQ-06 is
  still open, so do not read its presence as the decision having been made.
- `ros-jazzy-simulation-interfaces` 1.5.1, **message definitions only**. Per
  RQ-05 there is no Gazebo server on Harmonic that answers those services; the
  server implementation is Jetty-only. It is installed so
  `igvc_simulation_interface` still compiles, **not** because the services will
  work. Use `ros_gz_bridge` plus `gz service` for spawn and sim control.
- `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-rviz2`
- `mesa-utils`, `vulkan-tools` for diagnosis

Measured in the container: `gz sim --versions` reports **8.15.0**, which is the
same Gazebo build the Humble image ran. The simulator did not change when the
distro did, so every performance number in section 4 carries over exactly
rather than by analogy.

Size **5.57 GB on disk** as of 2026-09-15, up from 4.89 GB when the distro
moved to Jazzy. The extra ~0.7 GB is Nav2: `ros-jazzy-navigation2` and
`ros-jazzy-nav2-bringup`, plus `image-geometry` and `twist-stamper`, added so
the robot can navigate rather than only be driven (section 10). For comparison
the Humble image was 4.24 GB.

`docker image ls` and `docker inspect` disagree on size because the containerd
store unpacks layers lazily; `image ls` reports the on-disk figure used here.

It does **not** contain the workspace's Python/ML stack: **no `torch`, no
`ultralytics`, and no `models/yolopv2.pt`.** `numpy`, `cv2`, `cv_bridge`,
`message_filters` and `image_geometry` are all present, so the geometric half
of `igvc_lane_detection` runs and the ground-truth nodes run - but the lane
detector itself cannot, and that is the gap between today's ground-truth
navigation and real perception. Adding torch to this image is a deliberate
decision, not an oversight: it roughly triples the size, and perception work
has a home already in `igvc-humble-fused-drive`.

Base image note: this is `ros:jazzy-ros-base`, not the `osrf/ros:jazzy-desktop`
that the team standardised on for DevEnv. Desktop is roughly 3.5 GB against 800
MB, and `rviz2` and `xacro` are installed explicitly above, so nothing it would
have provided is missing. Disk is a live constraint here — `docker_data.vhdx`
never shrinks and Windows 11 Home has no Hyper-V to compact it.

---

## 6. Scripts

| Script | Purpose |
| --- | --- |
| `scripts/gazebo/render_check.sh` | GPU or llvmpipe. Exit 0 hardware, 1 software, 2 unknown. Run this first, always |
| `scripts/gazebo/render_check.sdf` | Minimal world, one camera plus one gpu_lidar, enough to force the render path |
| `scripts/gazebo/sensor_bench.sh` | Message rates and time-to-crash. `BENCH_TOPICS` and `WORLD` are env vars |
| `scripts/gazebo/three_camera_load.sdf` | Three ZED-placed rgbd cameras plus the RPLiDAR, at the URDF poses from section 3.3 |
| `scripts/gazebo/bridge_smoke_test.sh` | Proves `ros_gz` carries Image, LaserScan and Clock into ROS 2. Exit 0 on all pass |
| `scripts/gazebo/generate_igvc_world.py` | Builds `worlds/igvc_course.sdf` from `track_points.json`. See section 8.2 |
| `scripts/gazebo/start_sim.sh` | One command to bring the whole thing up inside the container: build, regenerate the world if the track data is newer, launch Gazebo plus RViz. Prints the teleop instructions. **`NAV=1` also starts Nav2 and the navigator, so the robot drives itself**; `RVIZ=0`, `HEADLESS=1`, `CAMERAS=all` |
| `scripts/gazebo/bringup_smoke_test.sh` | **The simulator and the contract.** 18 checks: every section 3.7 topic carries a message, the camera frame ids resolve in TF, and a commanded velocity moves the odometry in the right direction by the right distance against `gz` ground truth. Exit 0 only if the robot drives. `NAV=0` for the simulator alone, `RVIZ=1` for the GUI checks |
| `scripts/gazebo/autonomy_check.sh` | **Does the robot drive the course BY ITSELF.** Never publishes `/cmd_vel`; grades distance travelled, ground covered, deviation from the lane centreline, and whether it stayed on the ground slab. `DURATION`, `TOL`, `RVIZ` are env vars |
| `scripts/gazebo/sim_preflight.sh` | Sourced by the others. Refuses to start on top of a running simulator, because two on one ROS domain both publish `/clock`, `/odom` and `/tf` and produce results that look real and are not. `FORCE=1` cleans up instead of refusing |
| `scripts/gazebo/pose_logger.py` | Logs the robot's world pose at a fixed rate for `autonomy_check.sh`. Exists because `gz model -p` took ~20 s per call under a loaded simulator |

The scripts run inside the container, against the repo bind mount at
`/root/ros2_ws/src/IGVC_robot_2026`. `.gitattributes` already forces `eol=lf`
for `*.sh` and now for `*.sdf`, so they run directly off the mount with no
line-ending workaround:

```bash
bash /root/ros2_ws/src/IGVC_robot_2026/scripts/gazebo/render_check.sh
```

If someone checks the repo out with CRLF anyway, the symptom is
`bash: script.sh: cannot execute: required file not found`. Fix the checkout
rather than adding `tr -d '\r'` calls.

---

## 7. What this does not settle

- ~~**RQ-25, the branch question**~~ **Answered 2026-09-15.** The team lead says
  the Dockerfile on `main` of `IGVC_robot_2026` is what ran at competition. The
  trace, and the part of the question that is still open, are in
  [BRANCHES.md](BRANCHES.md) under "What actually ran at competition".
- ~~**RQ-03, the ZED path.**~~ **Answered 2026-09-15, sections 9 and 10.** The
  bridge publishes the section 3.7 contract names and the real 2026 navigation
  stack runs against them. The three `rgbd_camera` sensors in
  `three_camera_load.sdf` remain a load test, not the shim.
  [RQ03_AUDIT.md](RQ03_AUDIT.md) is the audit that fed it, and its closing
  section now records which of its findings held. The one that mattered most
  was a real frame-name bug that fails silently (9.4). Its claim that the
  container could not run any consumer was **four-fifths wrong** - four of the
  five packages were already present - and it corrects itself on both the fact
  and the method.
- **What the simulator still cannot do: perceive.** Lane lines and barrel
  positions come from `track_points.json`. The camera, depth, point cloud and
  lidar all publish and nothing plans on any of them, and running the real lane
  detector needs `torch` and the YOLOPv2 weights, neither of which is in the
  image. See 10.3 before describing this to anyone.
- **RQ-06**, and therefore whether `gz_ros2_control` gets used at all. It is
  installed since the move to Jazzy, so this is a design choice rather than a
  build problem - and it is now the **last** thing between the simulator and a
  full-fidelity control path. Nav2's twist currently goes straight to Gazebo's
  DiffDrive, so `/isaac_joint_cmd` is unbridged and `ros2_control` is not
  exercised at all. Outstanding item 19.
- **Native dual-boot versus WSL2.** Effectively answered: **stay on WSL2.** It
  reaches the GPU (section 3) and, launched from a WSL2 shell, it also gives a
  working GUI (section 3A). A native Ubuntu install would very likely be faster,
  since it would use the real NVIDIA GL driver instead of Mesa-on-D3D12, but
  that is now a performance optimisation, not a prerequisite, and it is not
  worth asking three Windows users to install a second operating system for.
  Revisit only if measured performance blocks real work.
- **The other two team machines.** The RTX 5080 Laptop and the Windows 10
  machine have still not been tested, and **native Linux has no verified path
  either** - `igvc_gazebo_linux` is written but unrun (`DOCKER_CHANGES.md`
  13.4). `GAZEBO_QUICKSTART.md` section 1A is written for exactly this gap. The Intel-adapter crash in particular depends
  on the iGPU, so it will differ per machine. `render_check.sh` exists so each
  member can settle it in one command.
## 8. The world and the launch file: a robot that drives

Added 2026-09-15. Before this, the image could render a test cube. These are
the two files `SIM_WORK_RESUMPTION.md` section 5 calls for, and with them the
robot spawns in the IGVC course and drives.

### 8.1 Run it

**Use `up -d`, not `run`.** This is the one thing worth getting right before a
demo. `docker compose run` creates a new container with a random name every
time, so a second `run` gives you a second, entirely separate simulator rather
than a second shell into the one you are looking at. `up` honours
`container_name: igvc_gazebo`, which is what `docker exec` needs.

```bash
# terminal 1, from a WSL2 shell - NOT PowerShell, see section 3A
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

`start_sim.sh` builds what it needs, regenerates the world if
`track_points.json` is newer than the world file, and starts Gazebo, the robot,
the bridge and RViz. Environment knobs: `RVIZ=0`, `HEADLESS=1`, `CAMERAS=all`.

```bash
# terminal 2, another WSL2 shell - drive it
docker exec -it igvc_gazebo bash
source /root/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keyboard teleop is deliberate. **The repo's `teleop.launch.py` cannot drive the
simulated robot**: it wants a physical joystick at `/dev/input/js0`, it pulls in
`motor_controllers.launch.py` with the `CanInterface` ros2_control stack that
Stage 1 does not use, and it remaps to `/diff_drive_controller/cmd_vel` rather
than the `/cmd_vel` that Gazebo's built-in DiffDrive listens on.
`teleop_twist_keyboard` needs no hardware and publishes straight to `/cmd_vel`.

Without teleop, the equivalent one-liner:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

### Doing it by hand

If you would rather not use the script, or something in it fails:

```bash
cd /root/ros2_ws
colcon build --symlink-install \
    --base-paths src/IGVC_robot_2026/src \
    --packages-select zed_description igvc_test_description igvc_test_bringup
source install/setup.bash
ros2 launch igvc_test_bringup gazebo_sim.launch.py rviz:=true
```

The colcon build is **required**, and is seconds rather than a real build: all
three packages are install-only. `test_robot.urdf.xacro` resolves its includes
with `$(find igvc_test_description)`, which needs the ament index.
`zed_description` is in the list because `igvc_test_description` depends on it,
and leaving it out fails the build outright.

### RViz

`rviz:=true` uses `config/gazebo_sim.rviz`, which is **not** the existing
`config.rviz`. That one is fixed to `base_link` and shows only the robot model
and TF, so a driving robot appears to stand still while the world slides past
it. The sim config is fixed to `odom` and adds the displays that show this is a
real ROS graph rather than a rendering: `/scan` returns, the `/front_zed/image`
camera, and an `/odom` trail.

It is a node in the same launch file rather than an include of
`rviz.launch.py`, because that file starts its own `robot_state_publisher` and
two of those publishing `/robot_description` and `/tf_static` against each
other is a confusing mess.

Those topic names are the raw Gazebo ones. When RQ-03 lands the ZED namespace
shim, the camera display needs updating with it. See section 8.7.

### 8.2 The world is generated, not converted

`scripts/gazebo/generate_igvc_world.py` reads
`IGVC_track_generator/track_points.json` and writes
`src/igvc_test_description/worlds/igvc_course.sdf`.

This follows the recommendation in section 3.2 of
`RESEARCH_QUESTIONS_AND_UNRESOLVED.md`: the generator already emits SVG,
OpenSCAD, STL and a JSON centreline, so the world never has to be recovered
from USD, which sidesteps the immature USD-to-SDF tooling problem entirely.

It also buys frame agreement for free. `centerline_m`, `obstacles_m` and
`robot_start_pose` are already in the `odom` frame that
`igvc_lane_detection`'s `navigator.py` and `gt_nav_bridge_node.py` consume, and
the launch file reads the spawn pose from the same file. So the course, the
robot's starting position and the ground-truth navigator share one coordinate
frame with **no alignment step and no scale factor to get wrong**.

`track.png` was deliberately not used as a ground texture. The JSON's own frame
notes say "x left (negated pixel x), y down, origin at image center", so a
texture needs two flips and an origin shift that are easy to get subtly wrong
and hard to notice. The metre arrays need none.

What it produces, at the defaults:

| | |
| --- | --- |
| Centreline | 78.8 m closed loop (259 ft), 1000 points resampled to 0.6 m |
| Lane lines | 123 left + 123 right segments, 3 in wide, offset for a 12 ft lane |
| Barrels | 8, cylinders at the `obstacles_m` radii |
| Ground | 39.5 x 33.6 m, deliberately dark for lane contrast |
| Total | 255 models, 135 KB |

### 8.3 Two decisions the research document had already flagged

**RQ-08, emissive lane paint. This was got wrong first, then fixed.** The Isaac
floor binds `track.png` to **both** `diffuseColor` and `emissiveColor`,
deliberately, so painted lines stay visible regardless of scene lighting. RQ-08
calls it "an easy thing to lose in conversion", and the first version of the
generator lost exactly that. Lane segments now carry an emissive term, default
0.55 via `--lane-emissive`. Not 1.0: full emissive blows out the camera and
makes the lines useless for thresholding. RQ-08 also asks for a *measurable*
check that a simulated camera sees lines the way a real one does. **That still
does not exist** and eyeballing it is not the same thing.

**RQ-11, obstacle collision, still OPEN and this is a decision.** Isaac's
generated `field.usd` gives obstacles no `CollisionAPI`, no `RigidBodyAPI` and
no mass, so **in Isaac the robot drives straight through them**. Whether that
was deliberate or an oversight is unknown. This world defaults to solid,
because an obstacle course the robot cannot hit does not test obstacle
avoidance. Pass `--no-barrel-collision` to match Isaac instead. Cylinders
rather than the STL meshes, because RQ-11 notes Gazebo mesh collision is
expensive and costs real-time factor directly. **Somebody should decide this
properly, and ideally fix whichever simulator is wrong.**

### 8.4 The collision meshes crash the physics engine

This is the one to remember, and it is not a Gazebo problem.

Four of the robot's collision meshes have **zero vertex normals**:

```text
caster_raceway_link_Part_2_Part_2.obj    209 vertices,  0 normals
caster_wheel_link__in_Caster...obj       500 vertices,  0 normals
left_wheel_link__in_Wheel_Stand_In...obj 528 vertices,  0 normals
right_wheel_link__in_Wheel_Stand_In..obj 528 vertices,  0 normals
```

dartsim reports `does not have a normal count [0] that matches its vertex count
[3168]. This submesh will be ignored!` and then segfaults in
`OdeMesh::fillArrays` on the empty result. `gz sim` dies on the first physics
step with exit 139. Note 528 x 6 = 3168 and 500 x 6 = 3000, exactly the counts
in the error.

Those four are precisely the parts that must collide for the robot to move: two
drive wheels, the caster wheel and its raceway. The chassis mesh is fine, 73324
vertices with 73324 normals.

The fix is in `test_robot_body.urdf.xacro` and is **simulation-only**, guarded
by `xacro:if value="$(arg sim)"`. The real robot still loads all four meshes,
verified by expanding the xacro both ways:

| Link | `sim:=true` | `sim:=false` |
| --- | --- | --- |
| `left_wheel_link` | cylinder r 0.1016, l 0.0640 | mesh |
| `right_wheel_link` | cylinder r 0.1016, l 0.0640 | mesh |
| `caster_wheel_link` | sphere r 0.0801 | mesh |
| `caster_raceway_link` | none | mesh |

Primitives are the better answer even ignoring the crash. A faceted mesh wheel
rolls on its flats and catches on its edges; a cylinder does not. A sphere is
the natural caster collider. The raceway is a bracket that never touches the
ground. The radii are not invented: 0.1016 and 0.0640 are the wheel mesh's own
measured bounding box, and 0.0801 comes from the joint-chain arithmetic in
`urdf/gazebo/gazebo_sim.urdf.xacro`.

**Worth raising with the team independently of Gazebo.** Those meshes are what
Isaac and any MoveIt planning would also load.

### 8.5 What the launch file does

`igvc_test_bringup/launch/gazebo_sim.launch.py` starts `gz sim` on the world,
publishes `robot_description` from the xacro with `sim:=true`, spawns the robot
at the JSON pose, and runs one `ros_gz_bridge` with 11 topics.

Bridge direction is the thing to get right: `[` is Gazebo to ROS, `]` is ROS to
Gazebo, `@` is both. `/cmd_vel` is the only one flowing **into** the simulator.

Three traps met while writing it, all of which fail confusingly:

1. **`robot_description` needs `ParameterValue(..., value_type=str)`.**
   Without it, launch tries to parse the URDF as YAML and dies.
2. **`package://` becomes `model://`** in sdformat's URDF conversion, so Gazebo
   needs a resource root containing a directory named literally
   `zed_description`. The checkout is `src/zed-description` with a **hyphen**
   while the package is `zed_description` with an **underscore**, so
   `model://` can never resolve against the source tree. `GZ_SIM_RESOURCE_PATH`
   is therefore built from `AMENT_PREFIX_PATH`, the install tree, where every
   package sits under its real name.
3. **An unresolved mesh URI degrades to a directory path**, which then
   segfaults the same ODE mesh loader as 8.4. A missing mesh and a normal-less
   mesh produce the same crash, so fix the paths before blaming the geometry.

### 8.6 Verified

`scripts/gazebo/bringup_smoke_test.sh`, through the compose service on the
RTX 5070 Ti Laptop, 2026-09-15. Run twice: headless, and again from a WSL2
shell with `RVIZ=1` so the Gazebo and RViz windows are exercised too.

**Superseded by the far stronger version in sections 9 and 10.** The current
test checks the section 3.7 contract topics, resolves camera frame ids through
TF, and compares odometry against `gz model -p` ground truth in both heading
and distance. Latest run: **18 of 18, 0 failures**, heading disagreement
0.13 degrees, distance ratio 0.9988.

```text
/clock /odom /scan /imu /joint_states /tf /front_zed/image   7 of 7 PASS
rviz2 node running                                           PASS
DRIVE TEST: PASS
```

The test commands a velocity and checks the odometry moves, because "gz sim
started" and "the topics exist" both pass on a badly broken setup: a world with
no robot still publishes `/clock`, and a robot with a wrong wheel radius still
publishes `/odom`.

Confirmed visually in the same run: the Gazebo window shows the course with
white lane lines and orange barrels at 97.4% real-time factor, and RViz shows
the robot model, `/scan` returns, an `/odom` trail and a camera panel in which
**the lane lines are clearly readable against the asphalt**. That last one is
the RQ-08 emissive fix doing its job.

#### An open question the two runs raised, and did not settle

Eight seconds at 0.6 m/s should be 4.8 m. The two runs disagree:

| Run | Distance | Implied duration |
| --- | --- | --- |
| headless | 5.466 m | 9.1 s |
| GUI | 4.622 m | 7.7 s |

The first looked like the robot continuing after the command burst ended,
which would mean Gazebo's built-in DiffDrive applies no `/cmd_vel` timeout
where `diff_drive_controller` does - a sim-versus-hardware divergence in the
unsafe direction. **The second run does not support that**: it came in below
4.8 m, which is what spin-up alone would produce.

So the likeliest explanation is jitter in when `ros2 topic echo --once`
connects and returns, not a missing timeout. **Unresolved either way.**

**A third attempt, 2026-09-15, also failed, and failed in a way worth
recording.** A rebuilt test drove 0.6 m/s for 8 s then watched for 5 s with no
commands. The robot kept moving, reproducibly, across two runs, and the result
was written up as "DiffDrive applies no `/cmd_vel` timeout". It was wrong. The
robot had driven off the edge of the 39.54 x 33.62 m ground slab - only about
9.5 m away on the spawn heading - and was falling, wheels spinning freely,
odometry counting up exactly as though it were driving. `world_z` read -15 m by
the third sample and -94,612 m by the end. **Free-spinning wheels and a held
command produce identical odometry**, so no odometry-only test can separate
them. `bringup_smoke_test.sh` now drives 0.4 m/s for 3 s, which keeps the robot
on the slab, and refuses to report a timeout verdict unless both samples were
taken with the wheels on the ground.

**A fourth measurement, clean this time, points the same way - and the question
is still not being called settled.** With the robot on the slab throughout,
nothing else publishing `/cmd_vel` (`nav2:=false`), and one simulator running,
it moved **3.767 m after the command burst ended**. That is far more than a
deceleration tail. It is NOT 4 s of travel though: each `ros2 topic echo
--once` sample costs seconds, so the real window was closer to 9 s, which at
the commanded 0.4 m/s is about what was measured. So the evidence says
DiffDrive holds the last command, and the honest position is that this has
been asserted and withdrawn twice already. A test that timestamps the last
command against the last odometry change, rather than inferring duration from
echo latency, would settle it. Outstanding item 17.

The
smoke test was never built to measure this and its timing is too loose to
settle it. If it matters - and for a competition robot a command-timeout
difference does - it needs a purpose-built test that timestamps the last
command and the last odometry change. Recorded as outstanding item 17.

### 8.7 What this did not give you - SUPERSEDED by sections 9 and 10

**Everything in this subsection was true when written and is no longer.** It is
kept because the rest of section 8 describes the state it belonged to.

~~The topic names are raw Gazebo, not the interface contract, so nothing
downstream runs - not lane detection, not Nav2, not the navigator.~~
**Closed 2026-09-15.** The bridge publishes the section 3.7 names (section 9)
and `gazebo_nav_test.launch.py` runs the real 2026 navigation stack against
them, with the robot driving the course by itself (section 10).

Two parts of it turned out not to be renames at all: Gazebo's odometry frame is
rotated by the spawn yaw (9.3), and the camera frame id was a one-word typo
naming a frame that does not exist in the TF tree (9.4).

Also still open: `sim_cameras:=all` has not been run against this world, so the
three-camera budget from section 4 is measured on a bare test scene rather than
the real course; and whether YOLOPv2 tolerates 640x360 remains unanswered,
which matters because the camera budget forces that resolution.

---


## 9. RQ-03: the interface contract, and the two things that were not renames

**Status: closed, 2026-09-15.** The simulation now publishes the topic names in
section 3.7 of `RESEARCH_QUESTIONS_AND_UNRESOLVED.md`, so `igvc_lane_detection`,
Nav2 and the navigator run against Gazebo unmodified. Section 8.7 said nothing
downstream ran; that is no longer true, and section 10 is the proof.

### 9.1 One YAML file is the contract

`src/igvc_test_bringup/config/gazebo_bridge.yaml` drives a single
`ros_gz_bridge` `parameter_bridge`. The YAML form matters and is not a style
choice: `parameter_bridge`'s `topic@ros_type[gz_type` argument syntax forces
the ROS name to equal the Gazebo name, so renaming would have to happen in a
launch remapping, invisible from the config. The YAML form takes
`ros_topic_name` and `gz_topic_name` separately, so the file **is** the
contract table rather than a description of one.

It also maps one Gazebo topic to two ROS names, verified 2026-09-15, which is
how `/joint_states` and `/isaac_joint_state` come from the same publisher at no
extra cost.

| Gazebo | ROS, the contract name |
| --- | --- |
| `/front_zed/image` | `/front_zed_camera_x/zed_node/rgb/color/rect/image` |
| `/front_zed/camera_info` | `/front_zed_camera_x/zed_node/rgb/color/rect/camera_info` |
| `/front_zed/depth_image` | `/front_zed_camera_x/zed_node/depth/depth_registered` |
| `/front_zed/points` | `/front_zed_camera_x/zed_node/point_cloud/cloud_registered` |
| `/sim/odom_body_frame` | `/odom` **and** `/front_zed_camera_x/zed_node/odom` |
| `/joint_states` | `/joint_states` **and** `/isaac_joint_state` |
| `/scan`, `/imu`, `/clock` | unchanged, already contract names |

All three cameras are listed unconditionally, so under the default
`sim_cameras:=front` the left and right topics are **advertised but never
publish**. Judge a camera by whether messages arrive, never by whether
`ros2 topic list` shows it.

This is the approach research pass 2 recommended for RQ-03, reached more
cheaply than it expected: pass 2 called for a `zed_sim_bridge` node
republishing each topic, and no node is needed, because the bridge can simply
be told the other name. That avoids a second serialisation of 640x360 RGB and
32FC1 depth at 15 Hz per camera.

### 9.2 What this is NOT

Stereolabs have publicly declined to support Gazebo and have no timeline. This
reproduces the **geometric** interface: the same topic names, message types,
frame ids and intrinsics. ZED NEURAL depth, visual odometry, confidence maps
and object detection have no Gazebo equivalent and are not approximated. Gazebo
depth is an exact ray-cast range with `+inf` past the far clip; ZED depth is a
neural estimate with its own failure modes on the texture-poor surfaces that
make up most of an IGVC course. **Do not report this as ZED parity.** Lane
perception work belongs on recorded SVO files and `yolopv2_bag/`.

### 9.3 Not a rename: Gazebo's odom frame is rotated by the spawn yaw

Gazebo's DiffDrive starts its odometry at identity, so the odom frame's axes
follow the robot's heading **at spawn**. The 2026 stack assumes the opposite.
`track_ground_truth_node._track_to_map` says so outright:

```text
# Odom/map frame is world-axis aligned with the spawn point at (0, 0).
# Only translate by -origin_offset; Isaac Sim's odom inherits world axes
# from the spawn pose, so the map must NOT be rotated by start yaw.
```

Measured on the IGVC course, 2026-09-15: spawned at world
`(-11.8817, 0.4511)` yaw `134.87 deg`, `/odom` read `(0.0000, 0.0000)` yaw
`0.00 deg` at the same instant. The two frames are 135 degrees apart.

Nothing in the TF tree can fix this, because `gt_nav_bridge_node` reads the
odometry pose **as a pose in the ground-truth grid's frame with no TF lookup at
all**. Fed raw Gazebo odometry it crops the local costmap from a point rotated
135 degrees away from the robot, every cycle, silently, with every topic green.

So `gazebo_odom_shim` rotates the odometry by the spawn yaw and owns
`odom -> base_link`. Gazebo's own odometry and TF outputs are renamed
`/sim/odom_body_frame` and `/sim/tf_body_frame` in the xacro so they cannot be
wired up by accident.

**Gazebo is not wrong here; Isaac was unusual.** Wheel odometry on the real
robot also starts at identity wherever the robot booted, because there is no
world frame to inherit axes from - that is what `map -> odom` absorbs. Isaac
published a ground-truth pose that happened to carry world axes and the stack
was written against the convenience.

The check that catches it is in `bringup_smoke_test.sh`: drive, stop, and
compare the odometry displacement against `gz model -p` ground truth. Three
runs, 2026-09-15:

```text
heading disagreement  0.13 deg      (a wrong odom frame shows up here)
distance ratio        0.9975, 0.9972, 0.9973
```

The ratio is a free second result: it corroborates `wheel_radius: 0.1016`. The
`0.2032` in `controllers.yaml` would have produced a ratio near 2.0. That is
evidence for team-lead question 2, from the simulator rather than from a tape
measure.

### 9.4 Not a rename: a one-word frame id typo that fails silently

`zed_macro.urdf.xacro:236` creates `${name}_left_camera_frame_optical`.
`gazebo_sim.urdf.xacro` stamped `${prefix}_zed_camera_x_left_camera_optical_frame`
- the Stereolabs upstream spelling, words transposed. **Every simulated camera
image carried a frame id naming a frame that does not exist in our TF tree.**

Nothing obvious catches it. The images publish, the topics carry data, the
frame id string looks exactly right in `ros2 topic echo`, and every name check
passes. `lane_detection.py:296-305` then falls back to **pinhole-only
projection** when the TF lookup fails, still publishes a costmap, and RViz
still draws plausible lane lines that are wrong by however far the camera sits
from `base_link`. The only tell is a warning throttled to once every 2 seconds.

This was found by a parallel audit, `docs/RQ03_AUDIT.md` item 2, and it is the
single most valuable thing that audit produced. It predates RQ-03 and would
have survived it.

`bringup_smoke_test.sh` now takes the frame id off the image and resolves it
through `tf2_echo base_link <frame>`. Checking the string against the xacro is
not a test; resolving it is.

---

## 10. The robot drives the course by itself

**Status: working, 2026-09-15.** `gazebo_nav_test.launch.py` is the Gazebo twin
of `isaac_nav_test.launch.py`, and unlike that file it is not commented out.
Start it, touch nothing, and the robot navigates the IGVC course.

```bash
# from a WSL2 shell, NOT PowerShell
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh   # NAV=1 for nav
# or the check that grades it
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh
```

### 10.1 What is actually in the loop

```text
track_points.json
   -> track_ground_truth_node    /lane_ground_truth   (reads JSON, no sim input)
   -> gt_nav_bridge_node         /lane_map, /lane_costmap  (+ obstacles stamped)
   -> igvc_navigator             extracts a lane path, calls FollowPath
   -> Nav2 controller_server     RegulatedPurePursuit
   -> velocity_smoother          acceleration limits
   -> collision_monitor          stop/slow polygons
   -> /cmd_vel                   Gazebo DiffDrive
```

Every node above is the 2026 code, unmodified, with the real robot's
`nav2_lane_follow_config.yaml`. The only Gazebo-specific file is
`config/gazebo_nav_test_nav2_overrides.yaml`, and it changes exactly two
things: which costmap layers load, and that the last hop publishes `/cmd_vel`
instead of `/diff_drive_controller/cmd_vel_unstamped`.

### 10.2 Measured

`scripts/gazebo/autonomy_check.sh`, RTX 5070 Ti Laptop, 2026-09-15, one
simulator, GUI and RViz both up, **no `/cmd_vel` published by the test at any
point**:

```text
samples              : 289          over 143.3 s of simulation time
distance travelled   : 122.9 m      entirely under its own control
furthest two points  : 29.9 m apart
centreline deviation : mean 0.72 m, max 1.79 m
final height         : 0.231 m, 0.000 off the spawn height
navigator            : aborts=4, grid=yes, path_reason=centreline has 35 points

MOVED       PASS      PROGRESS    PASS
IN LANE     PASS      ON THE SLAB PASS
```

**Two things in that block are now retracted. Read 10.2a before quoting it.**

**10.2a Retractions to the block above, 2026-09-17.**

*"The lane is about 3 m wide" is wrong, and it was the load-bearing
assumption.* The lane is **variable**, 10 to 20 ft, and the corridor
`track_ground_truth_node` actually plans in measures **2.766 m to 6.068 m**,
median 4.431 m. There is no constant lane width anywhere in the track
generator. See 10.8.

*"Four FollowPath aborts in 143 s" is withdrawn.* That figure is the `aborts=`
field of `/navigator/status`, which is `min(consecutive + 1, 4)`
(`navigator.py:1595`, `:1644`, `:1684`), is **reset to zero on success**
(`:1675`), and is also incremented on goal *rejection*. It saturates at 4 and
cannot tell 4 apart from 40. `autonomy_check.sh` samples it once at the end and
has never counted aborts. **Nothing in this repository counts aborts.** A later
run printed `aborts=1` with the same behaviour. The true total is unknown;
instrument `STATUS_ABORTED` results from the log before tuning anything.

*The deviation figures were sampled at about 2 Hz, which overstated the
margin.* `pose_logger.py` now samples at 10 Hz (about 15 Hz achieved). Three
runs at honest sampling reached max centreline deviations of **2.04, 1.95 and
1.98 m**. The behaviour did not regress; the measurement got honest. See 10.9
for what that does to the `IN LANE` gate.

### 10.3 It is NOT detecting the barrels, and that distinction matters

The robot drives around the barrels, and it is tempting to call that obstacle
detection. It is not. `obstacle_layer` is **disabled** in both costmaps;
`gt_nav_bridge_node` reads the barrel positions out of `track_points.json` and
stamps them into the grid as lethal cells, inflated by
`obstacle_inflate_radius_m`. The camera, the depth image, the point cloud and
the lidar are all publishing, and **nothing is planning on any of them.**

This is deliberate and it is what a ground-truth nav test is for: it proves the
navigation, control and simulation loop works, so that when perception is added
a failure can be attributed to perception. Claiming it as obstacle detection
would mean the first real perception bug looks like a regression in navigation.

The same applies to lane following. The robot stays between the lines because
the lines came out of `track_points.json`, not because anything looked at them.

### 10.4 The corridor, and the arithmetic corrected twice

**The earlier version of this section is withdrawn. It said the passable gap
was 0.87 m for a robot 0.70 m wide. Both numbers were wrong, and an earlier
edition of `GAZEBO_TODO.md` claimed this section had already retracted them
when it had not.** What follows is the corrected arithmetic.

**Neither width was a measurement.** 0.70 m is the **Nav2 footprint
parameter** (`nav2_lane_follow_config.yaml:250`, `:354`). The robot's collision
geometry is the chassis mesh, and under its `rpy` yaw of +90 degrees its
bounding box is **0.810 x 0.970 m** (`test_robot_body.urdf.xacro:203-208`). The
chassis is the widest part, not the wheels: the wheels sit 0.081 m (left) and
0.089 m (right) inside the chassis envelope. So Nav2 plans and inflates for a
robot **11 cm narrower than its own collision model**. Four different widths
are in circulation:

| Source | Width |
| --- | --- |
| URDF chassis collision bounding box | **0.810 m** |
| Nav2 footprint, both costmaps | 0.700 m |
| collision_monitor StopZone | 0.800 m |
| Design report | 0.762 m (2.5 ft) |

**The gap arithmetic, at the tightest barrel in the geometry Nav2 actually
plans in** (`track.png`, not the world file): 1.767 m clear, minus
`obstacle_inflate_radius_m` 0.30 = 1.467 m, minus the one-cell lethal
lane-boundary rim `gt_nav_bridge_node` stamps = **1.367 m of free cells**.

Nav2 then removes its inscribed radius from both sides:
`1.367 - 2 x 0.35 = 0.667 m`.

**Do not compare the robot's width against that 0.667 m. That was the second
error.** The inscribed radius has already been removed from both sides, so
0.667 m is the room available to the robot's **centre**, not to its body.
Comparing a full width against it counts the width twice. Taking the true
0.405 m half-width, the room is `1.367 - 0.81 = 0.557 m`, which is **positive:
the robot fits.**

What does still hold, and is the part worth acting on, is the weaker statement:
`1.367 - 2 x 0.75` (the `inflation_radius`) is **negative**, so **there is no
zero-cost cell anywhere in that gap.** Every path through it is a
non-zero-cost path. That is a cost-shape problem, not a geometry one.

**Treat this as a hypothesis for the abort behaviour, not a diagnosis.** The
abort count it was meant to explain is a saturating gauge (10.2a), so the thing
it predicts has never actually been measured.

Widening the footprint to the true 0.810 m raises the inscribed radius to
0.405 m and **makes the cost shape worse before it makes it correct.** That is
still probably the right trade, but it is a deliberate decision and it is not a
drive-by fix.

### 10.5 Three things that cost an hour, all worth knowing

**Nav2 starts 15 seconds after Gazebo, on purpose.** Nav2 brings ten lifecycle
nodes up in sequence and the whole bringup aborts if any one is slow -
`Failed to change state for node: smoother_server`, after which nothing
navigates and the cause is nowhere near the message. Launching it while Gazebo
is still loading meshes and rendering its first frame caused exactly that:
`controller_server` took 3.4 s of a 4 s bond timeout and the next node failed
outright. The lifecycle manager's `bond_timeout` cannot be raised from the
params file, because `navigation_no_docking.launch.py` constructs that node
with only `autostart` and `node_names`. So `nav2_delay` stops the race instead
of widening the window.

**Two simulators at once produce results that look real and are not.** A run
showed the robot driving well and then turning in circles at a barrel, plus the
lifecycle failure above. Both were investigated as navigation problems. Neither
was: `ps` showed **six `gz sim` processes, two `rviz2` and three
`parameter_bridge`** left over from earlier runs whose cleanup had not worked.

Two simulators on one ROS domain is not a degraded version of one. Both publish
`/clock`, so time moves in both directions. Both publish `/odom` and `/tf` for
a model named `igvc_robot`, so the TF tree holds two disagreeing answers and
consumers take whichever arrived last. Nav2 then plans against a pose that
teleports, which looks precisely like a controller tuning problem. The second
Gazebo is also rendering the same course on the same GPU.

`scripts/gazebo/sim_preflight.sh` now refuses to start on top of a running
simulator. The test scripts call it with `FORCE=1` and clean up first;
`start_sim.sh` refuses and tells you to restart the container. This is the same
family of mistake as `docker compose run` creating a new container each time,
and it earns the same treatment: check what is running before adding to it.

Its first version matched a list of node names and **missed five of them**,
leaving a second `gazebo_odom_shim` publishing `/odom` after a cleanup that
reported success - because killing `ros2 launch` does not reap its children,
they are reparented to init and carry on. It now matches on where the
executable lives (`/opt/ros/jazzy/lib/`, `/root/ros2_ws/install/`), which
catches every node in the stack and needs no maintenance when one is added.

**Two things must not drive the robot at once either.** `bringup_smoke_test.sh`
PART 2 and 3 command a velocity and measure the result, and Nav2 publishes to
the same `/cmd_vel`. Run together, the test found the robot already 21 m from
spawn before commanding anything, "coasting" 9.6 m after the burst, and
disagreeing with ground truth by 4.62 degrees instead of the usual 0.13. Every
one of those numbers was Nav2. The smoke test now passes `nav2:=false`;
`autonomy_check.sh` is where Nav2 is measured, and it measures it correctly by
never touching `/cmd_vel` at all.

One more false alarm worth recording, because it is the mirror image of the
useful kind. The smoke test reported the camera frame id missing from TF, and
it was present and correct the whole time - verified by hand with the identical
command. The cause was in the test, not the robot:

```bash
set -o pipefail                     # this script sets it
timeout 25 ros2 run tf2_ros tf2_echo base_link "$FID" 2>&1 | grep -q "Translation:"
```

`tf2_echo` prints once a second until the timeout. `grep -q` matches on the
first line and exits immediately, `tf2_echo` takes SIGPIPE, and **`pipefail`
then reports the whole pipeline as 141** - so a successful lookup reads as a
failure. Reduced:

```bash
set -o pipefail
(echo a; sleep 3; echo Translation:; sleep 3) | grep -q Translation:; echo $?
# -> 141
```

`set -o pipefail` and `grep -q` on a long-running producer are a silent
false-negative generator, and this repo's scripts all set `pipefail` because
`set -u` breaks the ROS setup scripts. The fix is to capture first and match
second. Both the TF check and the RViz check now do.

### 10.6 What is still not closed

- **`/isaac_joint_cmd` is not bridged, so ros2_control is not in the loop.**
  Nav2's twist goes straight to Gazebo's DiffDrive, which does the wheel
  kinematics itself. Joint limits, the controller update rate and its
  interaction with the physics step are therefore untested. That is RQ-06 and
  it needs a team decision, not a launch file.
- **`/fix` is not published.** Gazebo has a `navsat` sensor but the world needs
  `<spherical_coordinates>` first, and every sim config runs `gps_enabled:
  false`, as the Isaac path did.
- **Nothing perceives anything.** See 10.2.
- **The camera point cloud is rotated, and this is no longer a maybe.**
  Measured 2026-09-17: the cloud is emitted in the **body** convention, x
  forward, and stamped with the **optical** frame. `gz_frame_id` sets a header
  string; it does not rotate data. Through its own stamped frame the entire
  cloud collapses into a 12 cm slab with its fitted floor normal on **X**,
  89.78 degrees from +Z. Forced through the body frame it is a proper ground
  plane, normal on Z, 0.39 degrees from +Z, 100% inliers either way. Both
  offsets were predicted from the URDF beforehand (-0.096 and -0.229 m) and
  measured at -0.0994 and -0.2311 m, agreeing to 3.4 mm and 2.1 mm.
  `scripts/gazebo/cloud_frame_check.py` is the check, and it names which of
  four mistakes was made rather than just failing. **Not fixed**: the fix is
  not on `main` and is deliberately not in this release. See 10.10.

---

### 10.7 `render_check.sh` failed on a clean container

**Fixed 2026-09-17 (`d005781`). Retracted from the done list, not just
patched.** `GAZEBO_TODO.md` listed this script under "Done, and verified" and
the quickstart made it the first thing anyone runs on a new machine.

On this image `gz` comes from the ROS **vendor** packages
(`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`, gz-sim 8.15.0) and
`/root/.bashrc` does **not** source `setup.bash`. So the documented command
failed with `gz: No such file or directory` and printed `RESULT: UNKNOWN`,
which reads as a broken GPU rather than a missing PATH entry. Anyone following
the written setup on a new machine would have seen it.

The script now sources ROS itself. **The test that can fail:** run it from a
shell with nothing sourced. `which gz` prints `NOT ON PATH` and the script
still reports `HARDWARE RENDERING (NVIDIA)`, exit 0.

You therefore no longer need to source ROS by hand before calling it. Earlier
editions of the quickstart told you to; that instruction is harmless but
redundant.

### 10.8 The world now paints the course the planner actually plans in

**Before this change the robot's camera and its planner were looking at two
different courses.** `generate_igvc_world.py` painted a **constant 12 ft
(3.658 m)** lane into `igvc_course.sdf`. Nav2's corridor does not come from
that file at all: `track_ground_truth_node.py:339-374` derives it from contours
in `IGVC_track_generator/track.png`. Median disagreement **0.864 m**, maximum
**2.410 m**, and in **34.9%** of samples the planner's corridor was *narrower*
than the lane painted in Gazebo. **No ROS node ever reads `igvc_course.sdf`.**

**`track.png` is the designed course and the 12 ft constant was the outlier.**
`IGVC_track_generator/constants.py:116-117` sets `TRACK_WIDTH_MIN_FT = 10` and
`TRACK_WIDTH_MAX_FT = 20`, and `main.py:503` paints
`width = MIN + (MAX - MIN) * (0.5 + 0.5 sin(4 pi i/N))`: a sinusoid, two full
cycles per lap. That is a direct transcription of IGVC 2026 rules II.2, "track
width will vary from ten to twenty feet wide". **12 ft appears in neither the
rules nor the generator.**

`generate_igvc_world.py` now paints the variable profile by default, and it
**imports** the width constants from the track generator rather than copying
them, so there is one definition and a missing submodule is a hard error
instead of a guessed fallback. The imported values regenerate a byte-identical
world, which confirms the hand-mirrored numbers had been right.

**The test that can fail:** ray-cast the generated world's corridor from 200
centreline points and compare against `track.png`.

| World | corridor (m) | error vs `track.png` | within 0.15 m |
| --- | --- | --- | --- |
| **variable (new default)** | 2.734 to 5.778, median 4.210 | median **-0.043** | **90.0%** |
| constant 12 ft (old) | 3.541 to 3.639, median 3.592 | median **-0.813**, max 2.202 | 7.0% |
| `track.png` reference | 2.752 to 5.814, median 4.399 | | |

Median error improved by a factor of about 19. The residual 0.538 m maximum is
at curves, where an averaged-vertex-normal offset and a disc-stamped raster
necessarily differ.

`--lane-width-ft` is kept as an explicit override for a constant-width debug
course.

**The regenerated `igvc_course.sdf` is committed in the same change**, because
`start_sim.sh` only regenerates the world when `track_points.json` is newer
than it, and would otherwise leave every checkout on the old course.

**One fragility worth knowing.** `_get_lane_polygon_from_image` relies on
`track.png` having exactly the four-contour nesting its area-rank assumption
needs. Any extra white mark breaks the ranking silently, and fewer than four
contours falls through to a constant-width fallback with `lane_half_width_m`
defaulting to 0.25, which is a silent 0.5 m corridor.

### 10.9 Lane-boundary clearance, reported and not enforced

`autonomy_check.sh` measured distance to the **centreline** only, which cannot
say whether the robot stayed inside a lane whose width varies by a factor of
two. It now also reports the clearance between the chassis edge and the **inner
edge of the painted line**, evaluated at the **local** half-width, for both
candidate robot half-widths. The width profile is imported from
`generate_igvc_world.py` so there is one definition. **Pass criteria are
unchanged.** The premise was confirmed first: both painted lines are offset by
the same distance either side of `centerline_m`, so the centreline used for
"deviation" is the centre of the painted lane by construction.

**Yaw is not a refinement, it eats two thirds of the margin.** A fixed
half-width is the robot's lateral extent only when it is aligned with the lane.
Skewed, a corner leads, and the reach perpendicular to the lane is
`(W/2)|cos t| + (L/2)|sin t|`. For the 0.810 x 0.970 m chassis that runs from
0.405 m aligned to **0.632 m** at the worst angle, 50.1 degrees. Measured over
a full run the robot sits at a **mean 20.7 degrees** to the lane and peaks at
**65.3 degrees**, because the navigator is continuously re-aiming.

Pose sampling was also raised from about 2 Hz to 10 Hz (about 15 Hz achieved).
Re-scoring one 831-sample run at decreasing rates isolates the effect:

| Effective rate | Worst clearance | Max centreline deviation |
| --- | --- | --- |
| 15.8 Hz | **+0.093 m** | 2.04 m |
| 7.9 Hz | +0.116 m | 2.04 m |
| 4.0 Hz | +0.153 m | 2.04 m |
| 2.0 Hz | **+0.227 m** | 2.04 m |

The two quantities behave differently, and that is the finding: **the worst
clearance is a brief event and the worst deviation is not.** The old rate
overstated the clearance margin by 2.4x while leaving the deviation untouched.

Three runs with the rotated footprint at honest sampling:

| Run | Max centreline deviation | Worst clearance, chassis | Nav2 footprint | Over the line |
| --- | --- | --- | --- | --- |
| 4 | 2.04 m | **+0.093 m** | +0.135 m | 0.0% |
| 5 | 1.95 m | **+0.087 m** | +0.127 m | 0.0% |
| 6 | 1.98 m | **+0.078 m** | +0.119 m | 0.0% |

**The robot never put a corner over the paint in any run.** But the margin is
**8 to 9 cm**, not the 25 cm an earlier unrotated, under-sampled version of
this metric reported.

**The `IN LANE` gate is now visibly a coin flip, and it has deliberately not
been touched.** At honest sampling it produced 2.04 (FAIL), 1.95 (PASS) and
1.98 (PASS) against a flat 2.00 m tolerance. The behaviour did not regress: 47
to 51 m driven under its own control every time, on the slab throughout, and
0.0% of samples over the paint in all three. What changed is that the
measurement got honest. **The gate measures the wrong quantity**, a constant
tolerance against a lane whose width varies by a factor of two, and it sits
right on its own threshold. Raising `TOL` to make it green would be the wrong
fix twice over. **This needs a team decision.** Until it is made, read a
failing `IN LANE` alongside the clearance figures rather than as a regression.

### 10.10 Two guards and one forwarding fix

**`sim_cameras` used to validate nothing.** `gazebo_sim.urdf.xacro:209,212`
were two `xacro:if`s with no else, so any typo (`fron`, `front,left`, `true`)
silently produced a robot with **zero cameras** that launched cleanly. There
are now two guards, because there are two ways in: `gazebo_sim.launch.py`
raises a readable `RuntimeError` before xacro runs, and the xacro validates via
`['none','front','all'].index(cams)` for anyone invoking xacro by hand.

A wrong first attempt is worth recording: the xacro guard initially did
nothing, because **xacro evaluates properties lazily** and a validation
property that nothing reads is never evaluated. The fix was to make the
validated index the thing the dispatch switches on, so it cannot be skipped.

| `sim_cameras` | exit | rgbd cameras |
| --- | --- | --- |
| `front` | 0 | 1 |
| `all` | 0 | 3 |
| `none` | 0 | 0 |
| `fron`, `front,left`, `true` | **2** | rejected by the guard |

**Camera resolution now reaches the autonomy path.**
`gazebo_nav_test.launch.py` forwarded only six arguments, so
`sim_camera_width`, `sim_camera_height` and `sim_camera_hz` were unreachable on
the one launch file that runs the full stack. They are now declared and
forwarded, with defaults 640, 360 and 15, so nothing changes unless asked.

**What is NOT in this release.** The point-cloud frame fix (10.6) is
diagnosed, predicted and measured, and is **not** applied. Applying it means
re-enabling `obstacle_layer`, which connects a Nav2 consumer that has **never**
been connected in this simulator: `gazebo_nav_test_nav2_overrides.yaml:36-44`
replaces `plugins` with `["lane_layer", "inflation_layer"]` in both costmaps,
so the clouds have never reached `obstacle_layer` at all. That is a new system
under test, not a bug fix. Expect autonomy behaviour to change and re-baseline
all three gates afterwards.
