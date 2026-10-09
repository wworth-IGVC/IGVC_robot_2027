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
| Which Gazebo | **Harmonic**, `gz-sim` 8, and on Jazzy this is REP 2000's Tier 1 pairing, not an off-label choice |
| How | Docker image `igvc-gazebo-jazzy:latest`, built from `docker/Dockerfile.gazebo-jazzy` |
| Source build needed | **No, and now not even for `gz_ros2_control`**, which was the one exception on Humble |
| Third-party apt repo | **None.** Everything comes from packages.ros.org |
| GPU rendering | **Yes**, `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, but only with four specific settings |
| Three cameras at 30 Hz | **No.** At 1280x720/30 Hz you get ~7 Hz each |
| Three cameras, workable | **Yes at 640x360/15 Hz**: ~11 Hz each, physics at real time |
| A robot that drives | **Yes.** IGVC course world plus a spawn/bridge launch file. `bringup_smoke_test.sh`: 18 of 18 checks, 0 failures. See section 8 |
| Downstream nodes | **Yes.** The bridge publishes the section 3.7 contract names; the real 2026 ground-truth, navigator and Nav2 stack run against it. Section 9 |
| A robot that drives ITSELF | **Yes.** 122.9 m of autonomous driving, max 1.79 m off the lane centreline, no `/cmd_vel` from the test. Section 10 |
| Perception | **No, and do not imply otherwise.** Lanes and barrels come from `track_points.json`; nothing plans on the camera or lidar. Section 10.3 |

**New here? Read `docs/setup/WINDOWS.md` instead**, or the Word version of it.
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

### 2.2 Jazzy is where the code already was, but not the robot images

This split matters, and getting it wrong in either direction is easy.

**Already Jazzy, the code and the development environment:**

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

**Still Humble, the robot-side images.** Per the audit in section 5.4 of
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
  and `ros-jazzy-ros-gzfortress` **do not exist**, confirmed absent from the
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

**Installing it is not the same as adopting it.** RQ-06, `gz_ros2_control`
versus a `grr_hardware/GazeboDriveHardware` topic bridge mirroring
`IsaacDriveHardware`, is still open and still a team decision. It is present
only so that trying the `ros2_control` path costs nothing. Do not treat its
presence as the decision having been made.

### 2.5 The superseded Humble reasoning, kept for provenance

The original section 2 answered "Fortress or Harmonic, on Humble". Its
conclusion about the simulator was right and its evidence still holds; it was
simply asking the question one layer too high up.

It recorded that the research artifact recommended Fortress because Humble plus
Harmonic supposedly required building `ros_gz` from source, and that this
premise was **false**, `ros-humble-ros-gzharmonic` 0.244.12-3jammy and its
five siblings shipped as prebuilt jammy debs from packages.osrfoundation.org,
checked against the live index on 2026-09-10. That removed the maintainability
argument for Fortress and left its May 2027 EOL unanswered.

It also recorded the real Debian `Conflicts:` between the Fortress and Harmonic
flavours on Humble:

```text
ros-humble-ros-gzharmonic         Conflicts: ros-humble-ros-gz, ros-humble-ros-gzgarden
ros-humble-ros-gzharmonic-bridge  Conflicts: ros-humble-ros-gz-bridge, ros-humble-ros-gzgarden-bridge
```

Both findings are now historical. The Humble image was kept in
`docker/deprecated/` until 2026-09-29 and then deleted; git has it at
`88db437:docker/deprecated/Dockerfile.gazebo-harmonic`.

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

### Jazzy, 2026-09-15, current

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
factor of **99.77%**. Note the section 4 caveat, physics at real time says
nothing about sensor rates, which must be measured by counting messages.

### Humble, 2026-09-10, superseded baseline

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
top, the RTX 5080 Laptop and the Windows 10 machine remain unverified, and the
Intel-adapter crash in section 3 depends on which GPUs a laptop has.

---

## 5. What is in the image

Base `ros:jazzy-ros-base` (Ubuntu 24.04 noble), plus, **all from
packages.ros.org**, no third-party apt repo and no GPG key to fetch:

- `ros-jazzy-ros-gz` 1.0.24, the metapackage: `ros_gz_sim`, `ros_gz_bridge`,
  `ros_gz_image`, `ros_gz_interfaces`. Gazebo Harmonic itself arrives through
  its `ros-jazzy-gz-*-vendor` dependency chain.
- `ros-jazzy-gz-ros2-control` 1.2.20, installs
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
decision, not an oversight: it roughly triples the size. Perception work has
its own image, `igvc-dev-jazzy` (`docker/Dockerfile.dev-jazzy`): this image
plus torch and every workspace dependency, 14.9 GB. It replaced the Humble
`igvc-humble-fused-drive` on 2026-09-29, so nothing on the simulator's graph
is Humble any more (report section 20).

Base image note: this is `ros:jazzy-ros-base`, the same base the team's
DevEnv image `dev_env:9` builds from, not the `osrf/ros:jazzy-desktop` an
earlier DevEnv proposal used (that PR was closed unmerged). Desktop is roughly 3.5 GB against 800
MB, and `rviz2` and `xacro` are installed explicitly above, so nothing it would
have provided is missing. Disk is a live constraint here, `docker_data.vhdx`
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
| `scripts/gazebo/autonomy_check.sh` | **Does the robot drive the course BY ITSELF.** Never publishes `/cmd_vel`; grades distance travelled, ground covered, whether the chassis crossed the painted lane boundary (`LANE`), whether it touched a barrel (`BARRELS`), and whether it stayed on the ground slab. `DURATION`, `RVIZ`, `LAUNCH_ARGS` are env vars; `ANALYSE_ONLY=<track_log>` re-scores a saved run without a simulator (10.9) |
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
  13.4). `docs/setup/WINDOWS.md` section 1A is written for exactly this gap. The Intel-adapter crash in particular depends
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

### 10.9 Lane-boundary clearance: reported 2026-09-17, enforced as `LANE` from 2026-10-07

`autonomy_check.sh` measured distance to the **centreline** only, which cannot
say whether the robot stayed inside a lane whose width varies by a factor of
two. It now also reports the clearance between the chassis edge and the **inner
edge of the painted line**, evaluated at the **local** half-width, for both
candidate robot half-widths. The width profile is imported from
`generate_igvc_world.py` so there is one definition. **At that point the pass
criteria were unchanged; the gate changed on 2026-10-07, at the end of this
section.** The premise was confirmed first: both painted lines are offset by
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

**The `IN LANE` gate was visibly a coin flip** (2026-09-17). At honest
sampling it produced 2.04 (FAIL), 1.95 (PASS) and 1.98 (PASS) against a flat
2.00 m tolerance. The behaviour did not regress: 47 to 51 m driven under its
own control every time, on the slab throughout, and 0.0% of samples over the
paint in all three. What changed is that the measurement got honest. The gate
measured the wrong quantity, a constant tolerance against a lane whose width
varies by a factor of two, and it sat right on its own threshold. Raising
`TOL` would have been the wrong fix twice over, so it was left alone until the
decision below.

#### The change: `IN LANE` becomes `LANE` (2026-10-07, decision D4)

Liam delegated the decision and it was taken that day. `IN LANE` is gone.
**`LANE` fails if the chassis footprint's clearance to the painted boundary is
negative at any sample, and fails if the clearance is NOT MEASURED** (no yaw
column in the pose log, no width profile, or any non-finite sample). The
chassis figure is used, not Nav2's: it is the larger, stricter footprint. The
centreline deviation is still printed, as a number, and decides nothing.
`TOL` is no longer read; the script says so if it is set.

*Why this quantity.* The 2026 rules do not judge distance from the centre.
Page 11: "Crossing internal lines is not allowed and will be judged an E-Stop
end of run with penalty." Page 12 (the traffic violations): the distance is
measured to the furthest point where the final part of the vehicle crossed the
boundary edge; crossing with part of the vehicle still in bounds is "Careless
Driving", a 5 ft ticket with no E-stop. `LANE` follows page 11, so it is
stricter than that ticket. Read from the rules PDF on 2026-10-07. The sim lane
is 10 to 20 ft wide (half-width 1.52 to 3.05 m), so a flat 2.00 m was too loose
where it is narrow and too strict where it is wide.

*Offline scoring.* `ANALYSE_ONLY=<track_log> bash autonomy_check.sh` runs the
same Python block on a saved log and starts nothing. A log cannot carry what a live run measures, so
`CONTACT_LOG`, `CONTACT_SENSORS` and `FINAL_Z` come from the environment and
default to "not measured", which fails. There is one code path: the live run
calls the same block. `measurements/clearance_rate_sensitivity.sh` now uses it
too (it used to cut the block out with awk, and had been broken since the
contact arguments were added on 2026-10-01).

*Equivalence.* Re-scoring `gates_2026-10-07_stage24/3_track_log.txt` offline
reproduced the saved live output on every figure (86.8 m, span 29.7 m,
deviation mean 0.84 and max 1.80 m, yaw 17.3 and 59.2 deg, clearance +0.074 m
and +0.117 m, 0.0% over). Across all 18 saved runs below, MOVED, PROGRESS, ON
THE SLAB and BARRELS agreed with each saved output: 0 mismatches. (The first
pass showed one, an oldworld run whose world had no barrel sensors; the driver
had wrongly declared 8. The driver was fixed, not the gate.)

*Negative controls, offline, on a copy of that run (worst clearance +0.074 m):*

| Change to the pose log | Must | Result |
| --- | --- | --- |
| none (positive control) | PASS | **PASS**, +0.074 m |
| every pose moved 0.5 m away from the centreline, along the line from the nearest centreline point (perpendicular to the lane) | FAIL | **FAIL**, worst -0.371 m, 20.1% of samples over the paint |
| every pose moved +0.5 m in world y (weaker: along the lane where it runs along y) | FAIL | **FAIL**, worst -0.180 m, 2.7% over |
| yaw column removed | FAIL, NOT MEASURED | **FAIL**, NOT MEASURED |
| one sample's yaw set to nan | FAIL, NOT MEASURED | **FAIL**, NOT MEASURED (see below) |

The nan control found a real hole: `min()` and `< 0` both skip a NaN, so the
first version printed "worst +0.074 m" and would have passed a log with one
bad sample. Any non-finite sample now makes the whole clearance NOT MEASURED.

*Where it flips.* The same run with every pose pushed outward by a growing
amount (reported margin +0.074 m): PASS at 0.00, 0.05, 0.07, 0.08, 0.09 and
0.10 m; **FAIL from 0.11 m** (-0.003 m), and monotone beyond (-0.013 at 0.12,
-0.033 at 0.14, -0.093 at 0.20, -0.205 at 0.30, -0.371 at 0.50). So the gate
does flip, at a shift of about 0.11 m against a margin of 0.074 m; the 3 cm
between them is the resolution limit described below.

*The saved runs, old verdict against new* (`local-notes/oneshot_evidence/
lane_gate_2026-10-07/rescore_table.md`). Every ground-truth run passes `LANE`,
including the one that failed `IN LANE` at 2.07 m. No surprises.

| Run | Max centreline deviation | Old `IN LANE` (2.00 m) | Worst chassis clearance | New `LANE` |
| --- | --- | --- | --- | --- |
| 09-29 `gates_2026-09-29_dev_d1` run 5 | 1.86 m | PASS | +0.074 m | PASS |
| 09-29 `dev_d1` run 6 | 1.80 m | PASS | +0.027 m | PASS |
| 09-29 `dev_d1` run 7 | 1.91 m | PASS | +0.083 m | PASS |
| 09-29 `gates_2026-09-29_gz_g1` run 5 | 1.98 m | PASS | +0.035 m | PASS |
| 09-29 `gz_g1` run 6 | 1.86 m | PASS | +0.059 m | PASS |
| 09-29 `gz_g1` run 7 | 1.91 m | PASS | +0.062 m | PASS |
| 10-01 baseline | 1.66 m | PASS | +0.045 m | PASS |
| 10-01 stage 0 | 1.93 m | PASS | +0.081 m | PASS |
| 10-01 stage 1a verify, `gt` | **2.07 m** | **FAIL** | +0.084 m | **PASS** |
| 10-01 stage 1a, world without contact sensors | 1.85 m | PASS | +0.055 m | PASS |
| 10-01 stage 1a verify, same world | 1.87 m | PASS | +0.072 m | PASS |
| 10-07 stages 2 and 4 gates | 1.80 m | PASS | +0.074 m | PASS |
| *Void for lane purposes: the barrels were removed from the planner's grid and the robot ended pinned against one, so its odometry is not its position (10.11). `BARRELS` fails these.* | | | | |
| 10-01 stage 1a runs 1 and 4 (`none`) | 1.88, 1.90 m | PASS | +0.231, +0.278 m | PASS |
| 10-01 stage 1a runs 2 and 5 (`nav2_scan`) | 1.88, 1.87 m | PASS | +0.247, +0.215 m | PASS |
| 10-07 stage 1a verify (`none`) | 1.87 m | PASS | +0.254 m | PASS |
| 10-01 stage 1a verify, `none`: the robot never moved (0.0 m) | 0.00 m | PASS | +1.740 m | PASS (MOVED and PROGRESS fail it) |

*A live default run on the final script* (`gates2.sh lane_d4 autonomy`,
2026-10-07, the tree carrying the uncommitted stage 2 files): 88.8 m driven,
span 28.8 m, deviation mean 0.65 m and max 1.82 m, yaw to lane mean 15.2 and
max 49.8 deg, 1622 samples at 16.2 Hz, **worst chassis clearance +0.014 m**,
0.0% over the paint, no barrel contact (8 sensors live), on the slab, `aborts=2`,
real-time factor about 0.89. MOVED, PROGRESS, LANE, ON THE SLAB and BARRELS all
PASS; the leftover count was 0. Re-scoring its saved log offline gave the same
numbers.

*What the new gate can and cannot say. Read before trusting a pass or a fail.*

1. **The margin is thin.** The worst clearance of the 13 ground-truth runs
   above (including today's) runs from **+0.014 to +0.084 m**, median +0.062.
   The gate can fail, but an ordinary run can land within a centimetre of zero,
   so a `LANE` FAIL near zero is not by itself a regression and three passes do
   not show a comfortable margin. The cure is steering that keeps further from
   the paint, not a looser gate. Nobody has started that: it is the navigator
   and Nav2, which are protected.
2. **Resolution is about 3 cm.** The painted half-width is looked up by nearest
   centreline index, and `h` steps by up to 2.8 cm between neighbouring
   indices (mean 0.6 cm; 1000 points, 2.8 to 12 cm apart). In the sweep above the
   worst clearance went +0.004 m at a 0.07 m shift and +0.027 m at 0.08 m.
3. **It sees samples, not the path between them.** At 16 Hz and 0.9 m/s the
   poses are about 5 cm apart. Re-scoring today's log at lower rates gave
   +0.014 m (16.2 Hz), +0.052 m (8.1 Hz), +0.052 m (4.1 Hz) and +0.165 m
   (2.0 Hz): a brief crossing between samples can be missed, and the slower the
   sampling the more can be. The earlier finding about sampling (above) stands.
4. **Poses come from odometry through `gazebo_odom_shim`**, not from ground
   truth. After a barrel contact they are void, as before.

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

### 10.11 Barrel contacts, measured, and RQ-11 decided (2026-10-01)

**`autonomy_check.sh` could not see a barrel.** It read only the centreline
and the spawn pose from `track_points.json`, and it samples the robot's pose
from `/odom`, which a wheel slipping against a barrel corrupts at exactly the
moment that matters. Every claim that the robot "avoids the barrels" was
therefore inferred from where the ground-truth planner put them, never
measured. Before any perception goes in the loop, a touch has to be countable.

**RQ-11 is decided: barrels are static and solid** (Liam, 2026-10-01). The
IGVC rules penalise any touch ("Sideswipe/Obstacle Touch", 5 ft, the run
continues) and end the run on a crash or a displaced barrel (10 ft), so a test
only ever needs to know *whether* the robot touched a barrel, never what the
barrel did next. A barrel that can move would also change the course for the
rest of the run. Isaac's generated `field.usd` gives obstacles no collision at
all; `--no-barrel-collision` still reproduces that.

**How it is measured.** Each barrel in `igvc_course.sdf` carries a `contact`
sensor on its own collision (`generate_igvc_world.py`), all publishing to
`/sim/barrel_contacts`, bridged as `ros_gz_interfaces/msg/Contacts`
(`gazebo_bridge.yaml`), and the world loads `gz::sim::systems::Contact`. The
sensors are on the barrels rather than the robot because a contact sensor
watches one named collision, and every robot collision is lumped into
`base_link` (chassis, three camera bodies, lidar, antenna) plus the wheels and
caster; from the barrel side any part of the robot is reported. Static against
static (barrel and ground) is never computed, so the topic is silent until
something touches a barrel. `contact_logger.py` records each touch with its
simulation time, the barrel and the robot part.

**Silence is the dangerous result**, because a broken chain produces it too.
So `autonomy_check.sh` prints "none" only when Gazebo has contact sensors
publishing on `/sim/barrel_contacts` (counted as publishers with
`gz topic -i`, one per barrel) and the bridge offered the topic in ROS;
otherwise it prints NOT MEASURED and says which link is missing. The first
version of this guard used `gz topic -l`, and its negative control (the world
from `main`, which has no sensors) **passed it**: the topic is listed either
way, because the bridge *subscribes* to it. `gz topic -i` shows eight
publishers with the sensors and "No publishers" without.

**The test that can fail**, `scripts/gazebo/measurements/barrel_contact_check.sh`:

| Arm | Setup | Required | 2026-10-01 |
| --- | --- | --- | --- |
| HIT | spawn 1.2 m from barrel 0's centre facing it, drive 0.3 m/s for 5 s | barrel 0 reported | **1175 contact messages, barrel_0, by the chassis**; the robot stopped with its front face on the barrel's surface (x 5.322 m + 0.485 m = 5.807 m, the barrel's edge) |
| CLEAR | course start, the same command down the open lane | nothing reported | **0** |

A mistake it caught on the way: the first version put `<topic>` on the
`<sensor>`, which the Contact system ignores; it reads `<contact><topic>`. The
sensors then published under default names such as
`/world/igvc_course/model/barrel_0/link/link/sensor/contact/contact`, the
chosen topic stayed silent, and the HIT arm reported nothing while the robot
sat against the barrel. That is the failure this test exists for.

**`BARRELS` is a pass criterion of the default check**: a touch FAILS, and so
does being unable to measure touches. It was first added as reported only,
to leave the gate's meaning alone, and was changed the same day by what it
found. **A touch voids every other reading**, because they come from
odometry and odometry keeps counting while the wheels slip against a static
barrel. Measured (section 10.12): with the barrels removed from the planner's
grid, the robot drove into barrel 4 at 39 s and stayed pinned there to the
end of the run, and the check read a normal lap, 89.4 m, MOVED, PROGRESS and
IN LANE all PASS, while `gz model -p` put the robot 0.79 m from the barrel's
centre and **14.9 m from where odometry said**. The old gate passed a pinned
robot. A correct run loses nothing: the first default run with the sensors
(2026-10-01) touched no barrel, 89.9 m, worst lane clearance +0.081 m. So the
ground-truth robot's barrel avoidance is now measured, not assumed.

`autonomy_check.sh` also takes `LAUNCH_ARGS` now, appended to the launch, so
the same check runs on other configurations (section 10.12).

### 10.12 Nav2's obstacle layer cannot make `main`'s robot avoid a barrel (2026-10-01)

The plan for obstacle perception was to put `obstacle_layer` back with the
lidar as its only source (`/scan` is proven to see the barrels, 2026-09-24).
Reading the code first said it would not work, so it was measured before
anything was built on it.

**Why the code said no.** The navigator builds its path only from
`/lane_costmap` (`navigator.py:292`, centreline search `:1044-1267`) and sends
it straight to FollowPath, so Nav2's planner never runs; RPP runs with
`use_collision_detection: false` and cost-regulated scaling off
(`nav2_lane_follow_config.yaml:108`, `:115`); the collision monitor's only
source is `enabled: false` and it has no lidar source. Nothing that moves the
robot reads Nav2's costmaps. The ground-truth run avoids barrels only because
`gt_nav_bridge_node` stamps them into `/lane_costmap` itself.

**The switch.** `gazebo_nav_test.launch.py` takes `obstacle_source`:

| Value | The planner's barrels | Nav2 |
| --- | --- | --- |
| `gt` (default, unchanged) | stamped from `track_points.json` | lane and inflation layers |
| `none` | none: `gt_nav_bridge_node` gets a copy of the track file with `obstacles_m` emptied | as `gt` |
| `nav2_scan` | none | plus `obstacle_layer` on `/scan` alone (`gazebo_nav_test_nav2_obstacle_scan.yaml`, merged over the usual overrides at launch) |

The barrels stay in the world in every case: `gazebo_sim.launch.py` reads only
the spawn pose from the track file. Half of them are in the robot's path: with
the chassis half-width 0.405 m, a robot on the centreline would strike barrels
0, 2, 4 and 5.

**Measured**, two runs each, the full `autonomy_check.sh`:

| Condition | Run 1 | Run 2 |
| --- | --- | --- |
| `none` (the negative control) | touched barrel 4 at 39.1 s, pinned to the end | barrel 4 at 39.2 s, pinned |
| `nav2_scan` | barrel 4 at 38.5 s, pinned | barrel 4 at 39.3 s, pinned |

In both `nav2_scan` runs the layer loaded and subscribed in both costmaps
("Subscribed to Topics: lidar_scan"); the behaviour was identical to having no
obstacle perception at all. **So on `main` a barrel that only the lidar sees
is neither avoided nor stopped for.** Barrel 4 is simply the first of the four
in the path that the robot reaches; it never got further.

**A second, separate observation:** while `lane_layer` is on, Nav2's published
costmaps carry none of the lidar layer's marks. The costmaps with and without
the layer were identical (2000 lethal cells in the global costmap, 1269 in the
local one, in both). Switching `lane_layer` off at runtime revealed **24096**
lethal cells in the global costmap with the layer and **0** without it, so
the layer does mark and the lane layer hides it. The mechanism is not
established: Nav2 1.3.13's StaticLayer takes the maximum even in a rolling
window (`static_layer.cpp:476-479`), so it is not a plain overwrite. 24096 is
also far more than eight barrels: `inf_is_valid: true` turns "no return" into
`range_max - 0.0001` (`obstacle_layer.cpp:378`), 11.9999 m, inside the global
layer's 12.0 m `obstacle_max_range`, which would mark a phantom ring 12 m out.
Neither changes any decision here, because nothing that moves the robot reads
these costmaps; both are recorded for whoever next touches the real robot's
Nav2 configuration.

**What does work, on the competition branch.** `2026/more_diverging_changes`
(`67a6934`) adds `lidar_obstacle_costmap.py` (`/scan` to a decaying evidence
grid `/lidar_obstacle_map`) and its navigator overlays that grid's lethal cells
onto the region it searches for a path (branch `navigator.py:1270-1295`). That
is how the 2026 robot avoided obstacles in lane mode. Both files are in
`igvc_lane_detection`, a protected package, so bringing them to `main` is a
team decision; with it, `obstacle_source:=lidar_map` is the next step, judged
by this check with `none` as its negative control.

### 10.13 The point-cloud re-stamp, and why the cloud lost most of its frames (2026-10-07)

**The frame.** The cloud's data is in the camera's body convention while
`gz_frame_id` stamps it optical (10.6), and the same tag must stay optical for
the image, depth and camera_info. So the bridge delivers the cloud to
`/sim/<cam>_zed/points_optical_stamp` and `gazebo_cloud_restamp` republishes it
on the contract name stamped `<cam>_zed_camera_x_left_camera_frame`, one node
per camera, data untouched (it edits the `frame_id` inside the serialized
bytes and checks the first cloud decodes identically apart from that string).
The frame was right the first time; the rate was the problem.

| Frame test (640x360, `igvc_gazebo`) | Prediction | Result |
| --- | --- | --- |
| `cloud_frame_check.py` on the contract topic | PASS, floor on Z near -0.229 m | **PASS**, Z, -0.2311 m, 0.39 deg |
| Same, forced through the optical frame | FAIL, case C | **FAIL**, X, -0.0994 m, 89.78 deg |
| `--topic` on the raw private topic | FAIL, case C | **FAIL**, X, -0.0994 m, 89.78 deg |

**The rate, and where the loss really was.** The first version delivered 2.47
Hz of 10.51 Hz (23.5%) to a best-effort counter. The handoff suspected Python
and proposed a C++ node. Each step below changed one thing and was measured;
"in" and "out" are in the node, `RcvbufErrors` is the kernel's count of
datagrams dropped because a receiving socket's buffer was full
(`/proc/net/snmp`), and the real-time factor varied between runs from 0.62 to
0.91, so compare within a row, not down a column.

| Run | What changed | Result |
| --- | --- | --- |
| A | The node counts clouds in and out every 10 s | **In = out, every report, 10.0 to 12.9 Hz.** The node drops nothing. The counter saw raw 11.80 Hz and the re-stamped cloud 2.03 Hz (17%) |
| B1 | Raw and re-stamped counted in separate processes | raw 9.25 Hz, out 2.72 Hz. Not the counter sharing a thread |
| B3 | The counter subscribes RELIABLE | raw 10.89 Hz, out 11.20 Hz. The node publishes everything; a reliable reader gets it all |
| E3 | The node publishes best-effort instead | out 4.38 Hz against raw 11.60 Hz. Not the QoS pairing |
| E4 | No raw counter, only the out counter | out 3.80 Hz, and the node's own **input** fell to 6.7 to 7.9 Hz. The hop from the bridge to the node loses clouds too |
| C0 | Baseline with the kernel counter | out 3.60 Hz, **2198 `RcvbufErrors`** over 329,689 datagrams in the window |

So the node is not the bottleneck and a C++ node was not built: it would send
the same burst. (That is a judgement, not a measurement; no C++ node was tried.)
The cause is the transport. The compose files set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
(shared memory fails quietly, see `docker-compose.mac.yml`), so a 3.7 MB cloud crosses the loopback as a burst of datagrams sent back to
back. That overflows the receiving socket's buffer, the kernel drops
datagrams, and losing one loses the whole cloud for a best-effort reader. The
competition's `lane_segmentation` is one: it subscribes to the cloud with
`qos_profile_sensor_data` (`67a6934`, `lane_segmentation.py:628`). A reliable
reader gets repairs, which is why B3 saw everything.

**The fix, in configuration.** `config/fastdds_sim_cloud_pacing.xml` (renamed `fastdds_sim_camera_pacing.xml` on 2026-10-08, when it also took the images, 10.19) is a Fast
DDS profile with a flow controller (`cloud_pacer`: at most 131072 bytes every
1 ms, 128 MB/s against the 51 MB/s that 13.7 Hz of clouds needs) attached by
topic name to the six cloud writers (the raw and the re-stamped topic for each
camera) as `ASYNCHRONOUS`. `gazebo_sim.launch.py` hands it, with
`RMW_FASTRTPS_PUBLICATION_MODE=AUTO`, to the bridge and to the re-stamp nodes
only (`additional_env`), so no other process and no other writer in them
changes; the launch prints which it did. A profile the environment already
names is left alone, with a warning. Two other forms were tried. Without the env
var the rmw forces synchronous publication and Fast DDS refuses the flow
controller ("Cannot use a flow controller in synchronously publication mode"),
which killed `robot_state_publisher` in the first attempt (C1). A default
profile that paces every writer, with `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`,
also worked (C2), but then everything in those processes shares one pacer; whether
that delays `/clock` behind a cloud was not measured, and the per-topic form
avoids the question.

| Run | Setup | out (best-effort subscriber) | `RcvbufErrors` |
| --- | --- | --- | --- |
| C2 | all writers paced | 13.64 Hz, node in 13.7 Hz | 12 |
| C3 | per-topic profile, mode AUTO | 13.55 Hz, node in 13.4 to 13.7 Hz | 12 |

**The check had a flaw, found by the first run of the fix.** The first
`cloud_rate_check.py` counted the raw cloud as "in" with a second best-effort
subscription in the same process. That subscription starved itself: it counted
0.70 Hz while the re-stamped cloud arrived at 12.45 Hz, which is exactly the
camera's 15 Hz times the real-time factor (747 clouds in 49.8 s of sim time),
and out/raw = 17.8 "passed". A test that passes when its denominator collapses
is not a test. It now counts the cloud against the camera's own frames, the
per-frame `camera_info` (small, not lost) with the 15 Hz times sim seconds
alongside, and prints the raw cloud only on request, as information.

**Final measurements** (`stage2.sh final2`, then `rate_only.sh`, 2026-10-07;
evidence `local-notes/oneshot_evidence/stage2_final_2026-10-07/` and
`gates_2026-10-07_stage2_final/`):

| Test | Must | Result |
| --- | --- | --- |
| Frame arms (table above) | as predicted | **3 of 3** |
| `cloud_rate_check.py`, 60 s, best-effort | cloud >= 95% of `camera_info` | **PASS: 680 clouds against 680 frames (1.000)**; 673 by the clock (1.010); real-time factor 0.75; node in = out; `RcvbufErrors` 7 |
| Negative control: the plain UDP profile the competition compose sets (`fastdds_udp.xml`), so pacing is not applied (the launch says so) | FAIL | **FAIL: 77 clouds against 524 frames (0.147)**; `RcvbufErrors` 4125 |
| A RELIABLE consumer on the paced cloud, 45 s | all frames | **417 clouds = 15 Hz x 27.8 s sim** (real-time factor 0.62) |
| `fix_check.py`, and its 90 deg control | PASS, FAIL | **PASS** 0.002 m; **FAIL** 18.8 m |
| Gates on this tree | all pass | render **PASS**; smoke **18 passed, 0 failed**; autonomy **PASS**, 88.8 m, `LANE` worst clearance +0.086 m, no barrel contact, 0 leftovers |

**Not known, and not claimed.**

- One run (B2, the first RELIABLE-consumer run) ran at a real-time factor of
  0.06 (3.9 s of sim time in 60 s). It did not recur in B3 or in the paced
  reliable run, and its cause is not established. If a run suddenly crawls,
  that is the first thing to suspect.
- The RGB image (0.7 MB) and the depth image (0.9 MB) are subject to the same
  mechanism and are not paced here. Nothing has measured their loss. Count them
  with a best-effort subscriber before stage 3 relies on the camera.
- Whether the real robot's ZED cloud meets the same loss on its own network
  path, with `lane_segmentation` reading it best-effort, is not measured. A
  `RcvbufErrors` delta beside a run on the robot would say. A question for
  Aidan.

Reproduce: `bash scripts/gazebo/start_sim.sh` in one shell, then
`python3 scripts/gazebo/cloud_rate_check.py --seconds 60` in another.

### 10.14 GPS `/fix`, published and checked (2026-10-07)

The world now has `<spherical_coordinates>` (WGS84, ENU, heading 0, so world
+x is east and +y north) and the NavSat system; the sim xacro has a `navsat`
sensor on `front_gps_antenna_link` at 1 Hz with no noise, bridged to `/fix`
as `sensor_msgs/NavSatFix`, the u-blox driver's topic and frame. **The datum
is a placeholder**: the 2026 team's own simulator origin
(42.400510946, -83.130640432, from the competition branch's
`sim_gps_spoofer.py`), about 31 km from Oakland University. No surveyed
course coordinate exists in the repository; Aidan supplies one.

`scripts/gazebo/measurements/fix_check.py` takes a fix at rest, drives 3 s,
takes another, converts both to world metres through the datum, and compares
them with the antenna's ground truth (`gz model -p` plus the TF offset
rotated by yaw). Measured 2026-10-07:

| Run | Position error | Heading of the displacement | Verdict |
| --- | --- | --- | --- |
| `fix_check.py` | 0.002 m at both fixes | -0.00 deg over 1.359 m | **PASS** |
| `--rotate-deg 90` (control) | 18.35 and 19.96 m | +90.00 deg | **FAIL**, as it must |

Nothing in the simulator consumes `/fix` yet (`gps_enabled` is false in the
sim launches, and `main`'s localisation reads `/gps/fix`, not `/fix`: a
question for Aidan). This completes the sensor contract; it does not make
anything use GPS.

### 10.15 The competition branch is merged, on its own branch (2026-10-08)

`2026/more_diverging_changes` (`67a6934`), the code that ran at the 2026
competition, is merged into `sim-perception` on branch **`adopt-competition`**
(merge `91a4a25`, then `a14e074`, then `2ebe5b6`). `main` has not moved and
nothing is pushed. Liam approved it on 2026-10-07 and decided the two
conflicts on 2026-10-08. It is **not known good** (GPS failed completely at the
competition, lane detection missed lines), so anything wrong in it is fixed in
a separate commit with a test, never in the merge.

**Conflicts, two** (the 2026-09-24 count was one; `JAZZY_MIGRATION.md` 4):
`test_robot.urdf.xacro` (main's `sim:=true` include kept, the branch's robot
changes merged cleanly) and `igvc_test_bringup/package.xml` (main's rewritten
list kept, plus `imu_filter_madgwick` and `nav2_simple_commander`; `zed_wrapper`
dropped because rosdep has no rule for it on Jazzy). **Fixups:** `localization.py`
and its entry point kept (the simulator launches it; the branch deletes it);
all five submodule pointers the merge silently moved are back at main's
(`isaac/exts` stays at `375eddd1b`, the branch's alternative is `72ff9c21e`).

**Real-robot values the merge changes** (listed in the merge commit, expected):
`controllers.yaml` `wheel_radius` 0.2032 to 0.1016 (settles the open question
for the geometry; `wheel_separation` 0.5027 is unchanged and still differs from
the simulator's 0.57633); the RPLidar C1 mount 0.225 0 0.46 to 0.345 0 0.52 and
its mast; the side ZED X cameras from 95 to 45 degrees of yaw; and a **new
front ZED 2i link** at -0.215 0 0.248 that the robot USD does not have.
Navigation configs (`nav2_lane_follow_config.yaml`, `navigator_config.yaml`,
`lane_segmentation_config.yaml`) changed too.

**Build**, every package in `igvc_gazebo` (`colcon build --symlink-install
--base-paths src/IGVC_robot_2026/src`): **23 packages, exit 0**, after two
things that are not merge defects. `igvc_pointcloud_tools` needs `libpcl-dev`,
which no image installs; its verdict was Drop and it now has a `COLCON_IGNORE`
(`a14e074`). `ublox_gps` needs `libasio-dev`, absent from the simulator image
(an unchanged submodule, so `main` has the same property); it was installed in
the running container only. An existing build directory holds a dangling
symlink to the renamed `zed_multi.launch.py` and fails `igvc_test_bringup`
with "can't copy ... zed_multi.launch.py": delete that package's `build/` and
`install/` directories (a fresh clone is unaffected).

**Gates, before and after the merge** (default configuration, ground-truth
lanes and barrels; the freeze rule asks for three):

| Tree | Render | Smoke | Autonomy | Distance | `LANE` worst clearance | Yaw to lane mean / max |
| --- | --- | --- | --- | --- | --- | --- |
| `sim-perception` `3ba827b`, 2026-10-07 | PASS | 18/18 | PASS | 88.8 m | +0.086 m (and +0.014 m in the live D4 run) | 17.7 / 63.0 deg |
| merged, no override | PASS | 18/18 | **FAIL**, `MOVED` and `PROGRESS` | **0.0 m** | not meaningful | n/a |
| merged + sim override, run 1 | | | PASS | 90.1 m | +0.100 m | 9.3 / 32.8 deg |
| merged + override, run 2 (with render, smoke) | PASS | 18/18 | PASS | 89.4 m | +0.173 m | 8.8 / 32.4 deg |
| merged + override, run 3 | | | PASS | 89.0 m | +0.163 m | 8.7 / 29.5 deg |
| merged + override, run 4 | | | PASS | 90.1 m | +0.120 m | 9.0 / 31.3 deg |

No barrel contact in any run, 0 leftover processes. The merged navigator
tracks the lane more tightly and keeps further from the paint than `main`'s did.

**The one thing that had to change, in a sim-only file (`2ebe5b6`).** Without
it the robot never moved. The navigator logged "No valid lane path visible (pose
14 path heading 0.75rad > 0.75rad) - holding before reverse", then reversed 0.8
m, then held again. Measured: the simulated course turns right by about 45
degrees within 3 m of the start line (true centreline lateral offset -2.97 m at
3.2 m ahead). The branch's navigator limits every path's heading to
`max_path_heading_rad` (0.75 rad, 43 degrees). `_limit_path_heading` clamps the
slope to exactly `tan(limit)`; `_path_invalid_reason` then rejects any pose over
the limit, and smoothing and resampling push the clamped path a hair past it.
Mission state was `lane_follow`, so the mission gate was not involved.
`gazebo_nav_test.launch.py` now sets `max_path_heading_rad: 1.57`, the most the
validator accepts, which restores `main`'s old behaviour (no limit), in the
simulator only.

**Recorded, not fixed: a competition-code defect.** The clamp and the validator
disagree at the boundary, so any lane that turns more than 43 degrees inside the
navigator's look-ahead leaves the robot holding and reversing, on the real
robot as in the simulator. Whether 43 degrees is a sound limit is a separate
question. A fix needs a unit test on `_limit_path_heading` and
`_path_invalid_reason` together (a path at exactly the limit must be accepted)
and a decision on the number; it touches protected `igvc_lane_detection` and has
not been approved. The simulator now cannot see it.

**Not yet in the loop:** the branch's lidar obstacle map (`/lidar_obstacle_map`)
is not started by the simulator, so the navigator's overlay of it is inert and
barrels still come from `track_points.json`. That is the next section's work.

### 10.16 Obstacles from the lidar map, in the loop: measured, not yet passing (2026-10-08)

`gazebo_nav_test.launch.py obstacle_source:=lidar_map` removes the barrels from
the planner's ground-truth grid (as `none` does) and starts the competition's
`lidar_obstacle_costmap_node` with exactly the parameters
`igvc_fused_drive.launch.py:246-275` uses: `/scan` to `/lidar_obstacle_map` in
`odom`, 100 x 100 m at 0.10 m, inflation 0.20 m. The lidar mount comes from that
launch file's own `_top_lidar_mount_from_urdf()` (imported, not copied): x 0.345,
y 0, yaw 0, which equals TF `base_link -> top_rplidar_c1_link` exactly. The
merged navigator overlays the map's lethal cells onto the region it searches
(`navigator_config.yaml` `obstacle_map_topic: /lidar_obstacle_map`).

**Measured, one simulator, `autonomy_check.sh` with `LAUNCH_ARGS`:**

| Run | Barrels | MOVED / PROGRESS / ON THE SLAB | `LANE` | Map live (a counter, 30 s) |
| --- | --- | --- | --- | --- |
| C3 `lidar_map` 1 | **PASS**, no contact (8 sensors) | PASS, 87.4 m | **FAIL**, -0.390 m, 2.6% over | 1 publisher, 63 msgs |
| C3 `lidar_map` 2 | **PASS** | PASS, 86.8 m | **FAIL**, -0.696 m, 5.4% over | 1 publisher, 67 msgs |
| C3 `lidar_map` 3 | **PASS** | PASS, 87.0 m | **FAIL**, -0.144 m, 1.8% over | 1 publisher, 73 msgs |
| C1 `none` (negative control, same session) | **FAIL**: touched `barrel_4` at 35.3 s, about 76 s in contact | readings void after 35.3 s | (void) | n/a |

So the competition's lidar obstacle path, in the loop, **does avoid every
barrel** (the same robot without it pins on barrel 4), **but it crosses the
paint** while doing so. Where (`scripts/gazebo/measurements/lane_crossings.py`,
the gate's own arithmetic, per span of negative clearance):
every run squeezes between barrel 4 and the right-hand line at 34 to 37 s of
sim time, 0.24 to 0.31 m from the barrel's surface and 0.07 to 0.14 m over the
paint; runs 1 and 2 also cross once more on the left (near barrel 3, -0.390 m;
near barrel 1, -0.696 m). With barrels from ground truth the same navigator
never crossed the paint in four laps (worst clearance +0.100 to +0.173 m,
10.15), so the difference is the lidar map, not the navigator alone.

**Not tuned, by rule.** Candidate causes, each with the test that separates it,
for the next session: the map's 0.20 m inflation is smaller than the
ground-truth bridge's, so the gap beside barrel 4 looks passable (vary
`inflate_radius_m` in the sim only and see whether the robot chooses the other
side); the map decays and ray-clears, so a barrel seen side-on may thin out
(count lethal cells around barrel 4 while passing it); and the navigator does
not count the painted lines as harder than barrels (compare the costs it
assigns). All three touch competition code or its parameters, which this task
does not change.

Reproduce: `LAUNCH_ARGS="obstacle_source:=lidar_map" bash
scripts/gazebo/autonomy_check.sh`. Evidence:
`local-notes/oneshot_evidence/task4_lidar_map_2026-10-08/`.

### 10.17 Does the simulator reproduce "missed lines"? A baseline for the competition detector (2026-10-08)

The lane detector that competed (the merged `lane_segmentation.py`: YOLOPv2 on
the front camera, lane pixels projected through the organised point cloud), run
open loop in `igvc_dev_jazzy` beside the simulator while the robot drove itself
on ground truth. Hough and the ground truth were scored in the same drive by
`lane_eval_sim.py` (0.25 m tolerance). No tuning, no code change. The point cloud
was the re-stamped, paced one (10.13), stamped
`front_zed_camera_x_left_camera_frame`, with TF found.

| Detector | Hit rate | Coverage, observed | Coverage, whole window | Placement error, median / 90th |
| --- | --- | --- | --- | --- |
| **Competition YOLOPv2, 2026-10-08** | **0.424** | **0.845** | 0.390 | **0.300 / 0.827 m** |
| `main`'s YOLOPv2, 2026-09-29 (for comparison) | 0.166 | 0.762 | 0.266 | 0.768 / 1.683 m |
| Hough, same drive | 0.419 | 0.490 | 0.066 | 0.321 / 2.326 m |
| Ground truth, same drive (the ceiling of this metric) | 0.676 | 0.996 | 0.996 | 0.096 / 1.555 m |

**Conditions, which matter:** real-time factor **0.41** over the window (the
simulator, the detector on the GPU, Hough and four evaluators in one
container; nvidia-smi showed P8), and the cloud reached a best-effort
subscriber at **0.794** of the camera's frames, with 9171 receive-buffer errors:
the pacing of 10.13 is not enough at this load. The detector logged 7 "Dropping
stale synced frame ... lane model/input is behind camera rate" (ages 2.0 to
5.7 s) and twice "yolopv2 produced 0 projected points" while its raw mask held
932 and 1219 lane pixels ("post_mask_px lane=0").

**So "missed lines" is reproducible in kind but not yet attributable.** The
competition detector misses a large share of the paint (coverage of the whole
window 0.390, against the ground truth's 0.996) while what it does report is
mostly in the right place. Candidate causes, each with the measurement that
separates it:

1. **Throughput.** Stale frames are dropped at an RTF of 0.41. Separate: count
   frames received, processed and dropped by the node, alone in the container
   at RTF near 1, against the same numbers under today's load.
2. **The projection drops what the model saw.** Raw lane pixels became zero
   after masking. Separate: log, per frame, raw lane pixels and projected
   points, and vary one gate at a time offline on saved frames: depth range
   (`min/max_detection_depth_m` 0.3 to 5.0 m), the ground-plane z gates, the
   chassis mask.
3. **Range.** A 5.0 m depth limit cannot cover paint further out, which caps
   coverage of the whole window whatever the model does. Separate: coverage
   within 5 m against beyond 5 m in the same run.
4. **Cloud loss under load** (0.794 here). Separate: the synced image and cloud
   pairs per second the node receives, with and without the evaluators.
5. **Resolution and lens.** 640x360 against thresholds tuned on the real camera.
   Separate: one run at 1280x720 (`sim_camera_width/height`), one camera.

Evidence: `local-notes/oneshot_evidence/task6_comp_yolopv2_2026-10-08/`;
harness `local-notes/oneshot_evidence/scripts/yolo_eval2.sh`.

### 10.18 GPS: making the 2026 mission planner work as intended (2026-10-08)

GPS failed completely at the 2026 competition. Liam's instruction: take what it
was evidently meant to do as the goal, fix one link at a time, each with a
simulator test that fails on the old code first. The intended function: the
waypoints are entered before the run, the robot lane-follows, and the GPS
mission starts **by itself** when the robot reaches the start waypoint. Where
the waypoints go: `docs/GPS_WAYPOINTS.md`.

**(a) The start-area gate (2026-10-08).** The planner compared the first three
DIGITS of latitude and longitude with the start waypoint's: anything inside a
tenth of a degree (about 11 km) matched, a fix 20 cm away across a digit
boundary did not, and the shipped placeholder waypoints `(0, 0)` rejected every
real fix with a message about "prefixes (+424, -831)". It also took the first
fix as the odom origin, true only if the robot had not moved since boot. Now
the gate is a distance (`gps_start_match_radius_m`, 10 m), the anchor pairs the
fix with the robot's odom position at that moment, and placeholders are named
in the log.

`scripts/gazebo/measurements/gps_start_check.sh` (robot spawned at the course
centre facing east, start waypoint 8 m ahead, gate 4 m, the real
`mission_planner_node` on the simulator's `/fix` and odometry):

| Arm | Must | Old code (`PLANNER_PYTHONPATH`) | New code |
| --- | --- | --- | --- |
| P shipped placeholders | never anchor, and say why | FAIL: did not anchor, log only about prefixes | **PASS** |
| F 8 m away, at rest | not anchor | **FAIL: anchored on the first fix** | **PASS** |
| N driven at the waypoint | anchor within the gate, obstacles within 0.5 m of truth | FAIL: anchored 8 m away, before moving | **PASS**: at 4.0 m, after 4.0 m of driving, obstacles 0.24 m off |

Unit tests (`test_mission_planner_gps_gate.py`, rewritten for the distance
gate): 12 pass on the new code, 11 fail on the old. The 0.22 to 0.24 m residual
is the GPS antenna's offset from `base_link` (0.177, -0.133 m), which the
planner does not correct; recorded, not fixed.

**(b) The mission starts by itself (2026-10-08).** `auto_start` was false, and
the config's own comment said an operator would call `~/start_mission` on the
start line; the rules allow no operator during a run. It is now true by
default (code and config), still gated by (a): `_begin()` waits for the start
waypoint and the anchor. `scripts/gazebo/measurements/gps_mission_check.sh`
runs the full stack (Nav2 and the lane navigator on ground truth) with the real
planner, the start waypoint 15 m of lane ahead, the end 10 m after it, a 4 m
gate, and never calls a service:

| Code | `/mission/state` | Verdict |
| --- | --- | --- |
| before 5b (`auto_start: false`) | `idle` only; the planner anchored when the robot reached the gate and then waited | **FAIL** |
| after 5b | `idle` -> `active` at 3.3 m from the start waypoint -> `lane_follow` 31 s later; Nav2 accepted the goal; "end waypoint reached"; robot 1.0 m from the end waypoint | **PASS** |

That is the first GPS waypoint leg driven end to end in the simulator. It
says nothing yet about the real robot's heading problem: in the simulator the
odometry axes happen to point east and north (`gazebo_odom_shim`), which (c)
addresses.

**(c) The heading: odometry's axes are not east and north (2026-10-08).** The
planner put each waypoint's east and north metres straight into odom x and y.
Real odometry starts at identity, so its x axis points wherever the robot faced
at boot, and every waypoint lands rotated by the start heading. The simulator
hid this: `gazebo_odom_shim` rotates odometry onto the world axes. New, sim
only: `gazebo_sim.launch.py odom_axes:=robot` makes the shim a pass-through, so
the odometry axes follow the spawn heading as on the real robot. New in the
planner: it fits its `/fix` track to its odom track (a 2D rotation over the
displacements, at least 10 m of travel, residual under 2 m), anchors only once
the estimate exists, and rotates waypoints by it; `odom_yaw_fit_enabled:=false`
restores the old behaviour.

`scripts/gazebo/measurements/gps_heading_check.sh`: robot at the course centre,
`odom_axes:=robot`, four start headings, driving 14 m to the start waypoint,
obstacles compared with ground truth in that odom frame (pass under 0.5 m):

| Heading | Before (5b code) | After | After, fit disabled (control) |
| --- | --- | --- | --- |
| 0 deg (east) | PASS, 0.24 m | **PASS**, 0.25 m | PASS, 0.24 m |
| 90 deg | **FAIL**, 8.62 / 12.57 m | **PASS**, 0.25 m | FAIL, 9.24 / 13.21 m |
| 180 deg | **FAIL**, 12.76 / 18.47 m | **PASS**, 0.25 m | FAIL, 12.10 / 17.79 m |
| 135 deg (the course's start) | **FAIL**, 11.52 / 16.74 m | **PASS**, 0.26 m | FAIL, 11.11 / 16.31 m |

The estimates, from the planner's log: -0.1, 89.9, 179.9 and 134.9 deg, residual
0.00 to 0.01 m over 10.1 to 10.5 m of travel (the simulator's GPS has no
noise; a real receiver's metre or two will widen this, which is what the
residual limit is for). That is the prediction exactly: before the fix only the
heading that happens to face east passes. Unit tests 18/18 (the fit at four
rotations, anchoring with and without it). The (a) and (b) checks pass again on
the final code: (a) now anchors 2.1 m past the start waypoint, after 10.1 m of
driving, waiting for the heading; (b) idle -> active -> lane_follow, 1.1 m from
the end waypoint.

**Still not tested anywhere:** a real receiver's noise; a whole mission with
No Man's Land, the ramp and judge-style waypoints (needs "course v2"); and the
real robot's launch path, where the u-blox driver comes only from
`sensor_launch.launch.py` (its include in `igvc_fused_drive.launch.py` is
commented out; a proposal is in `docs/GPS_WAYPOINTS.md`, not made).

### 10.19 The RGB and depth images were losing most of their frames too; now paced (2026-10-08)

Measured like the cloud (10.13): `scripts/gazebo/camera_rate_check.py`, one
large stream per run (a second large subscriber starves itself), a best-effort
subscriber, counted against the camera's own `camera_info`, the kernel's
`RcvbufErrors` beside it, 45 s per stream, front camera at 640x360 and 15 Hz.

| Stream (size) | Unpaced: received / camera frames | `RcvbufErrors` | Paced | `RcvbufErrors` |
| --- | --- | --- | --- | --- |
| RGB image (0.69 MB) | **0.214** (2.51 Hz) | 2144 | **1.000** (11.07 Hz) | 15 |
| depth image (0.92 MB) | **0.345** (3.98 Hz) | 2022 | **1.002** (11.40 Hz) | 40 |
| point cloud (5.53 MB, paced since 10.13) | 1.000 | 20 | 1.000 | 11 |

So until today every best-effort consumer of the simulated RGB image, which
includes both lane detectors, received about a fifth of the frames. That is a
candidate cause for the "missed lines" baseline of 10.17, measured with the
images unpaced, and Hough's and YOLOPv2's numbers there should be re-measured.

The fix is the same mechanism: the profile, renamed
`config/fastdds_sim_camera_pacing.xml`, now also lists the image and depth
writers (all three cameras), all sharing one FIFO flow controller,
`camera_pacer`, at 131072 bytes per ms, so the three streams queue behind one
another instead of bursting together. Gates on this tree: render PASS, smoke
18/18, autonomy PASS (89.5 m, `LANE` +0.182 m, no barrel contact).

**A build trap fixed on the way.** A symlink-install build directory keeps a
link for every file it ever installed, so renaming or deleting a launch or
config file leaves a dangling link and `colcon build` fails ("can't copy ...:
doesn't exist or not a regular file") on every workspace built before the
change. It happened twice today (the merge renamed `zed_multi.launch.py`; this
rename). `igvc_build` now deletes dangling links, only those, in the packages
it builds; the rename above then built without any manual cleanup.

Evidence: `local-notes/oneshot_evidence/task6b_images_2026-10-08/`.

### 10.20 The lidar-map lane crossing: two causes found, a clearance rule tried and not kept, not passing (2026-10-09)

Task: make `obstacle_source:=lidar_map` pass `BARRELS` and `LANE` three times,
with `none` still failing `BARRELS` (10.16 measured LANE failing 3 of 3).
Diagnosed one parameter at a time, then a fix was tried in competition code
behind a parameter whose default is the old behaviour. **Result: not passing.**
Two causes are found and evidenced. The fix tried removes the barrel 1, 3 and 4
crossings in the runs that finished but creates a new one at the barrel 7 corner
(worse as the value rises), so it is **not committed**: it is saved as
`local-notes/oneshot_evidence/p1_lidar_lane_2026-10-09/min_clearance_attempt.patch`
with its six tests. Only the diagnosis tooling, one test and these notes are in
the repository. Evidence: `local-notes/oneshot_evidence/p1_lidar_lane_2026-10-09/`
and `lidar_lane_diag_2026-10-08/`. All runs: one simulator, `autonomy_check.sh`
with `LAUNCH_ARGS`, `BARRELS` read from the contact sensors.

**What was measured (every row `lidar_map`; `BARRELS` PASS in every run that moved):**

| Inflation | Lidar min range | Navigator clearance | Result |
| --- | --- | --- | --- |
| 0.20 m (the competition's) | 0.15 | off | 10.16: `LANE` FAIL 3 of 3, barrel 4 every run |
| 0.30 m | 0.15 | off | run 1: 90 m, `LANE` FAIL -0.269 m at barrel 1. Run 2 (same setting, a day later): **never left the start** |
| 0.35 and 0.40 m | 0.15 | off | **never left the start**, both. Identical, see below |
| 0.50 m | 0.15 | off | never left the start (2026-10-08) |
| 0.30 m | 0.25 | off | moved; `LANE` FAIL -1.005 m, crossings far from any barrel (not diagnosed) |
| 0.30 m, second run | 0.25 | off | moved; `LANE` FAIL -0.451 m at barrel 1 |
| 0.40 m | 0.25 | off | moved; `LANE` FAIL, ended off the slab (-22.3 m), `ON THE SLAB` FAIL |
| 0.20 m | 0.25 | 0.45 m, first version | never left the start: no path (see "built-in trap" below) |
| 0.20 m | 0.25 | 0.45 m | run 1: stalled over the left paint near barrel 7 from 66.7 s, 49.3 m, `LANE` FAIL -0.205 m. Run 2: 89.4 m, `LANE` FAIL -0.181 m, 2.4% of samples over, one span near barrel 7 only |
| 0.20 m | 0.25 | 0.65 m (the chassis half-diagonal) | run 1: 86.9 m, left the lane after the barrel 7 corner at 66.8 s and ended 34 m from the course, off the slab: `LANE` FAIL, `ON THE SLAB` FAIL. A second run was stopped for the time box |

**Cause 1, the robot's own body is in the map.** The lidar's `inflate_radius_m`
is applied as `ceil(radius / 0.10 m)` cells, so 0.35 and 0.40 are both 4 cells
and the table's identical rows are not a coincidence; the inflation cannot be
tuned finer than 0.10 m. More important: at the start, `/scan` has 4 finite rays
at 161 to 164 degrees, ranges 0.111 to 0.134 m, which is the robot behind the
lidar (mount x 0.345 m). The map node ignores returns under `min_range_m` 0.15,
but the sensor's 1 cm noise puts some of them over it, and the sim's parameters
(copied from `igvc_fused_drive.launch.py`: `hit_weight` 5, `hit_threshold` 4) make
one hit lethal. The cell lands about 0.2 m ahead of `base_link`, inside the
footprint; with the map's inflation it is a disc of radius 0.2, 0.3 or 0.4 m on
the robot. At 4 cells it walls the robot in (57 lethal cells at the origin in
`pic_infl040/picture.txt`), at 3 cells it does so some of the time, which is why
the same setting moved on one day and not the next. Tested by changing one
number: `min_range_m` 0.15 to 0.25 (`lidar_min_range_m` launch argument) and the
robot starts at 0.30 and 0.40 (0.35 was not rerun; it is the same 4 cells as 0.40). Unit test:
`test_lidar_costmap_body_return.py` (a 0.16 m body return enters the map at
0.15 and not at 0.25). **Open:** whether the real RPLidar C1 sees its own mount;
the competition value is unchanged, the 0.25 is a simulator launch override.

**Cause 2, the navigator accepts any gap.** `pic_infl030/` and
`ser_mr025_030/series.txt` (robot-frame pictures of the lidar map and the lane
grid) show the robot beside barrel 1 at 70 s with the paint 0.1 to 0.2 m to its
left and the barrel's lethal blob to its right: a free corridor about 0.5 m wide,
for a 0.810 m chassis. `navigator.py` `_extract_centerline` treats a corridor one
cell wide as traversable (its comment: clearance is allowed below the robot's
half-width "without falsely blocking narrow gaps"), so a route through the gap
costs only a little more than the wide side. The Nav2 costmaps do not help:
`nav2_lane_follow_config.yaml` lists `plugins: ["lane_layer", "inflation_layer"]`
for both costmaps (lines 253 and 315); `lidar_obstacle_layer` appears only in a
comment (line 233). So in the 2026 stack the lidar map reaches the path only
through the navigator's overlay, which is the stage with no half-width. The
ground-truth bridge stamps the whole disk plus 0.30 m and the same gap exists
there; the difference is that the barrel is fully known from afar (the lidar sees
an arc, more as the robot nears), and `scripts/gazebo/measurements/barrel_gaps.py`
prints the corridors per barrel.

**The fix tried, and why it is not kept: `min_clearance_m` in the navigator.**
Cells the centerline search may use must be at least that far from every lethal
cell (paint and overlaid obstacles), measured to the lethal cell's edge. Default
**0.0 is the old rule**, so the old code stays a control. Three details, each from
a failure: cells within 1 m ahead of `base_link` are exempt (the live
`/lane_costmap` is lethal across its full width for the first 0.3 m or more ahead
of the robot, and without the exemption a clearance of 0.45 m left no cell within
the 0.8 m start-cell search, so the robot never left the start: "first version" in
the table); if no cell beyond the exempt zone qualifies the old rule applies; and
the value was moved from the half-width 0.45 m to the chassis half-diagonal 0.65 m
because `LANE` scores the rotated 0.810 x 0.970 m footprint (half-diagonal 0.632
m). Tests (`test_navigator_min_clearance.py`, 6): on the old code the one that
asserts a 0.5 m gap is refused fails and the five controls pass; on the new code
all 6 pass. **Live result: worse where it matters.** With the rule, no run
crossed at barrel 1, 3 or 4, but every run crossed the left paint at the barrel 7
corner (-0.205 and -0.181 m at 0.45 m; off the course at 0.65 m), and no run
without the rule crossed there. Hypothesis, not tested: the lane grid leaves
the space beyond the paint UNKNOWN, which the search treats as traversable, so
once cells near the paint are excluded the only cells with room are outside the
lane. The next attempt should compute the clearance rule over FREE cells only
(unknown stays available to the old rule), then run the same unit tests and 3
live runs.

**Alternatives rejected, with reasons.** (a) Inflation alone: no value works,
because it is quantised to 0.10 m cells and 2 cells fail barrel 4, 3 cells are
inconsistent and 4 cells start only once the body is out of the map. (b) A
dilation of the overlaid cells (the ground-truth bridge's 0.30 m): it adds to
the lidar map's own inflation and narrows a gap by that amount from one side, so
it blocks a gap only when the dilation exceeds the gap; the clearance rule treats
paint and obstacles alike and states the chassis size once. (c) Putting
`/lidar_obstacle_map` into the Nav2 costmaps: the navigator builds the path and
the controller follows it, so costmap awareness would stop the robot, not reroute
it; not tested. (d) Raising `hit_threshold`: unrelated to either cause.

**Found on the way, not fixed.** 21 of the 33 tests in
`test_navigator_waypoint_generation.py` and `test_centerline_navigator.py` fail on
the shipped competition code: the shared fixture lays the grid out with forward
along rows, the navigator reads forward along columns, and the shell lacks
`_obstacle_grid`. The navigator rejects paths with "path heading 1.57rad >
1.57rad" (the clamp and validator mismatch of 10.15) often in the later part of a
lidar-map run. The 0.30 m run with `min_range` 0.25 crossed the paint 5.8 m from
the nearest barrel, which no barrel-side fix explains.

**Next.** (1) Retry the clearance rule over FREE cells only (patch above is the
starting point), 3 live runs, `none` control. (2) Separately, find why the 0.30 m
run with `min_range` 0.25 crossed the paint 5.8 m from any barrel (rejections
"path heading 1.57rad" in its log); that is not a barrel-side problem and may be
the navigator's clamp and validator mismatch of 10.15. (3) Ask whether the real
RPLidar C1 returns its own mount; if it does not, the 0.25 m override is a
simulator-only correction and the competition's 0.15 m stays. Not done: the
shared lidar parameter file for `igvc_fused_drive.launch.py` and the simulator,
because the fix it was meant to carry does not exist yet and Liam asked on
2026-10-08 that real-robot launch files not be edited.

**Controls rerun after the experiments** (working tree with the navigator patch
applied and its parameter at the default 0.0; the repository itself does not
contain the patch): ground truth `autonomy_check.sh` PASS, 89.4 m, `LANE` worst
clearance +0.169 m, `BARRELS` PASS; `obstacle_source:=none` `BARRELS` FAIL (touched
`barrel_4`), as it must. Evidence: `verify_gt_none/`. Real-time factor was not
recorded in these runs; laptop numbers, not Jetson numbers.

Reproduce a row: `LAUNCH_ARGS="obstacle_source:=lidar_map
lidar_min_range_m:=0.25 lidar_inflate_radius_m:=0.30" bash
scripts/gazebo/autonomy_check.sh`. Corridors per barrel: `python3
scripts/gazebo/measurements/barrel_gaps.py 0.20 0.30 0.40` (inside the container).


### 10.21 `lane_source`: the switch is in the tree, ground truth unchanged, the barrels now their own overlay (2026-10-09)

`gazebo_nav_test.launch.py` gains `lane_source:=gt|hough|yolopv2` (default `gt`),
`lane_warmup`, `yolopv2_weights` and `gt_outputs`, and `obstacle_source:=gt_map`.
Nothing here runs a camera yet; this is the plumbing and the proof that it does
not disturb the ground-truth loop. The code was written offline (task P3s, against
fake ROS modules) and applied and checked here (task P3a). Evidence:
`local-notes/oneshot_evidence/P3a_lane_source_2026-10-09/`.

What the switch does: in a camera mode the ground-truth bridge's `/lane_map` and
`/lane_costmap` move to `/gt/lane_map` and `/gt/lane_costmap`, so the navigator and
Nav2 cannot read them; `obstacle_source:=gt` under a camera is promoted to
`gt_map` with a printed `NOTE:`; `gt_obstacle_map_node` (new, sim-only,
`igvc_test_bringup`) publishes the barrels, and only the barrels, on
`/lidar_obstacle_map` in `odom` with the bridge's own rasterising arithmetic, the
same topic the navigator reads for the lidar map, so the navigator cannot tell
them apart; the detectors (`lane_detection_node`, `lane_segmentation_node`) start
with the protected configs verbatim; Nav2 starts at `nav2_delay` and the navigator
after `lane_warmup`. YOLOPv2 refuses to start without `torch` or weights and names
`igvc_dev_jazzy`. Default arguments give the old graph.

| Test | Result |
| --- | --- |
| U1 `gt_obstacle_map_check.py` (node against the real bridge code and `track_points.json`) | PASS: 8 barrels parsed identically, 946 lethal cells in both, 0 differing; control A (inflation +0.10 m) differs in 334 cells, control B (spawn offset ignored) in 1893 |
| Packaging: plain (non-symlink) `colcon build` of `igvc_test_bringup` | PASS: `ros2 pkg executables` lists `gt_obstacle_map_node`, it is installed executable, the module imports from `install/`, the installed launch file carries `lane_source`; the symlink workspace was restored and lists it too |
| L1 `lane_source:=bogus`, L4 `obstacle_source:=bogus`, L2 `lane_source:=yolopv2` in `igvc_gazebo` | PASS, each exits 1 in 0 to 1 s with the stated message and spawns nothing. **L3 (bad weights path) not run**: it needs `igvc_dev_jazzy`; do it at the start of P6 |
| G0a default graph unchanged | PASS after normalising (see below): 24 nodes and 90 topics before and after. Control: `obstacle_source:=gt_map` differs by exactly `gt_obstacle_map_node` and its publisher on `/lidar_obstacle_map` (25 nodes) |
| N4 `lane_eval_sim.py --expect-node` | PASS: `lane_detection_node` on a default run exits 3 ("`/lane_map` is not published by lane_detection_node alone"); `gt_nav_bridge_node` proceeds and scores |
| Probe smoke (`lane_source_probe.py`, default configuration, 60 sim s) | 12 PASS, 0 FAIL, 0 INCONCLUSIVE (its first contact with real rclpy) |

**G0a needed a normalisation, and the helper's filter was wrong.** The raw
`diff -r` of the two dumps was not empty, but every differing line was a `GID:`
line or one of two anonymous `transform_listener_impl_<hex>` node names.
`graph_dump.sh` removes lines matching `Endpoint GID`; this ROS prints `GID:`, so
it removed nothing, and the tf listener names carry a random suffix. Nothing else
differed. `local-notes/oneshot_evidence/scripts/g0a_norm.sh` drops `GID:` lines,
renames the listeners and sorts; it touches nothing else (node and topic names and
types, QoS and who-publishes-what are compared as dumped). Raw and normalised
dumps are in the evidence folder.

**C1: `obstacle_source:=gt_map`, `autonomy_check.sh`, 3 of 3 PASS.** One
simulator, laptop numbers (RTX 5070 Ti Laptop), not Jetson numbers. The real-time
factor is the mean of Gazebo `/stats` sampled every 10 s beside the run, including
the start-up samples that read 0 (so the mean understates the steady state).

| Run | Distance | `LANE` worst clearance | `BARRELS` | Arc advanced (`course_progress.py --arc-only`) | RTF mean |
| --- | --- | --- | --- | --- | --- |
| `gt_map` 1 (probe beside it) | 88.9 m | +0.190 m | PASS, 8 sensors live | 91.36 m | 0.40 |
| `gt_map` 2 | 89.4 m | +0.220 m | PASS, 8 live | 91.66 m | 0.33 |
| `gt_map` 3 | 90.0 m | +0.102 m | PASS, 8 live | 93.64 m | 0.47 |
| default 1 | **0.0 m, MOVED and PROGRESS FAIL** | n/a | PASS | n/a | 0.49 |
| default 2 (rerun) | 88.9 m | +0.144 m | PASS, 8 live | 92.39 m | 0.42 |
| default 3 (rerun) | 86.9 m | +0.123 m | PASS, 8 live | 88.95 m | 0.45 |

`gt_map` distances are within 5 percent of the passing default runs' (86.9 to
88.9 m). The probe beside `gt_map` 1: W6 PASS, 946 lethal cells against 945
expected from the 8 discs, all 8 disc centres lethal; W1 on `/lidar_obstacle_map`
FAILED once on a naming race, below.

**The first default run did not move, and I cannot prove the switch did not cause
it.** `lifecycle_manager_navigation` aborted Nav2's bring-up (`Failed to change
state for node: collision_monitor ... async_send_request fail`), after which
nothing navigates: the start-up race `gazebo_nav_test.launch.py` already
documents ("the whole sequence aborts if any one of them is slow"). The two
reruns under the same harness passed, the graph is identical to the old one, and
the default path's timers are unchanged, so it is recorded as 1 flake in 3
default runs (and 0 in 3 `gt_map` runs); the earlier default run of the day on
this branch also passed. If it recurs, look at the harness first (the RTF sampler
runs `gz topic -e` every 10 s beside the launch during bring-up).

**The probe's W1 had a bug the first real run found.** On `/lidar_obstacle_map` it
saw `['_NODE_NAME_UNKNOWN_', 'gt_obstacle_map_node']` with at most 1 publisher at
once over 674 samples: rclpy names an endpoint that way until its node has
announced itself. `record_graph` now keeps unresolved names out of the set of
seen names (they still count toward "at most one at once"). A four-case check
(`scripts/probe_w1_check.py` in the evidence folder): the transient passes; two
publishers at once, a wrong name and a name that never resolves all fail. A live
`ros2 topic info -v /lidar_obstacle_map` during a `gt_map` run showed one
publisher (`gt_obstacle_map_node`) and one subscriber (`igvc_navigator`).

**Reference for the camera runs (D12).** Arc advanced by the passing
ground-truth-lane runs of this tree: 91.36, 91.66, 93.64 (`gt_map`), 92.39, 88.95
(default). **REF = 91.66 m** (median of all five; 92.03 m with the three `gt_map`
runs and the first passing default run). It is more than one lap: the centreline
is 78.83 m and the 100 s window covers about 1.16 laps, so the fraction of a lap
reaches 1.16 to 1.19 and a run's arc is not bounded by 78.83. The 0.8 gate is
therefore 73.3 m of arc.

**Not verified.** Any camera mode (`hough`, `yolopv2`), whether the navigator
reacts to the overlay as to barrels in the lane grid in a camera run (U1 proves
equal cells, C1 in ground-truth lanes proves equal behaviour there), L3, and the
probe's W2, W3, W4 geometry in a camera mode. The `helpers/*.sh` staging aids
are not in the repository.

Reproduce: `LAUNCH_ARGS="obstacle_source:=gt_map" bash
scripts/gazebo/autonomy_check.sh`; U1: `python3
scripts/gazebo/measurements/gt_obstacle_map_check.py`.
