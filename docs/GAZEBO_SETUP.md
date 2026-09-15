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

Run it **from a WSL2 shell, not PowerShell** (see section 3A):

```bash
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
docker compose -f docker-compose.windows.yml build igvc_gazebo
docker compose -f docker-compose.windows.yml run --rm igvc_gazebo
```

Then, inside the container, before trusting anything:

```bash
bash /root/ros2_ws/src/IGVC_robot_2026/scripts/gazebo/render_check.sh
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

Size **1023 MB downloaded, 4.89 GB on disk**. The two figures disagree because
the containerd image store unpacks layers lazily; `inspect` reports the former
and `image ls` the latter. For comparison the Humble image was 920 MB / 4.24
GB, so Jazzy costs about 100 MB more, which is `gz_ros2_control` and the
`ros2_control` stack it pulls.

It does **not** contain the workspace's Python/ML stack; it is a simulator, not
a replacement for `igvc-humble-fused-drive`.

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

- **RQ-25, the branch question, still outranks everything.** This is simulator
  infrastructure; it is not an opinion about which commit to convert.
- **RQ-03, the ZED path.** Nothing here replaces the ZED SDK. The three
  `rgbd_camera` sensors in `three_camera_load.sdf` are a load test, not the
  shim.
- **RQ-06**, and therefore whether `gz_ros2_control` gets built at all.
- **Native dual-boot versus WSL2.** Effectively answered: **stay on WSL2.** It
  reaches the GPU (section 3) and, launched from a WSL2 shell, it also gives a
  working GUI (section 3A). A native Ubuntu install would very likely be faster,
  since it would use the real NVIDIA GL driver instead of Mesa-on-D3D12, but
  that is now a performance optimisation, not a prerequisite, and it is not
  worth asking three Windows users to install a second operating system for.
  Revisit only if measured performance blocks real work.
- **The other two team machines.** The RTX 5080 Laptop and the Windows 10
  machine have not been tested. The Intel-adapter crash in particular depends
  on the iGPU, so it will differ per machine. `render_check.sh` exists so each
  member can settle it in one command.
