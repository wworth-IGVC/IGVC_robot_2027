# Gazebo setup: which version, how it is installed, and proof it uses the GPU

**Created:** 2026-09-10
**Machine under test:** RTX 5070 Ti Laptop GPU, 12227 MiB, driver 610.88,
Intel Core Ultra 9 275HX, Windows 11, Docker Desktop 29.7.2 (WSL2 backend)

This closes **RQ-01** (is Humble + Harmonic viable) and **RQ-02** (does Gazebo
render on the GPU) with measurements rather than sources. Both were P0 and both
blocked everything else.

---

## 1. Short version

| Question | Answer |
| --- | --- |
| Which Gazebo | **Harmonic**, `gz-sim` 8.15.0 |
| How | Docker image `igvc-gazebo-harmonic:latest`, built from `docker/Dockerfile.gazebo-harmonic` |
| Source build needed | **No.** Prebuilt Harmonic `ros_gz` binaries exist for Humble |
| GPU rendering | **Yes**, `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, but only with four specific settings |
| Three cameras at 30 Hz | **No.** At 1280x720/30 Hz you get ~7 Hz each |
| Three cameras, workable | **Yes at 640x360/15 Hz**: ~11 Hz each, physics at real time |

Run it:

```powershell
docker compose -f docker-compose.windows.yml build igvc_gazebo
docker compose -f docker-compose.windows.yml run --rm igvc_gazebo
```

Then, inside the container, before trusting anything:

```bash
bash /root/ros2_ws/src/IGVC_robot_2026/scripts/gazebo/render_check.sh
```

---

## 2. RQ-01: Harmonic, and the research artifact is wrong about source builds

**Decision: ROS 2 Humble + Gazebo Harmonic.** This keeps the working
recommendation from `RESEARCH_QUESTIONS_AND_UNRESOLVED.md` section 2.2 and
declines the research artifact's recommendation of Fortress.

The artifact recommended Fortress on the grounds that Humble + Harmonic
"works because the Gazebo team added a Humble/Harmonic pairing to `ros_gz`, but
you must build `ros_gz` from source on Ubuntu 22.04; there is no ROS-official
apt binary".

**The premise is false.** Queried against the live apt index on 2026-09-10:

```text
http://packages.osrfoundation.org/gazebo/ubuntu-stable/dists/jammy/main/binary-amd64/Packages.gz

ros-humble-ros-gzharmonic             0.244.12-3jammy
ros-humble-ros-gzharmonic-bridge      0.244.12-3jammy
ros-humble-ros-gzharmonic-image       0.244.12-3jammy
ros-humble-ros-gzharmonic-interfaces  0.244.12-3jammy
ros-humble-ros-gzharmonic-sim         0.244.12-3jammy
ros-humble-ros-gzharmonic-sim-demos   0.244.12-3jammy
gz-harmonic                           1.0.0-1~jammy
```

These are prebuilt jammy `.deb`s. They are "non-ROS-official" only in the sense
that osrfoundation hosts them instead of packages.ros.org. **No colcon build,
no vendored `actuator_msgs`, nothing for a new team member to reproduce.** The
artifact's `ros_gz` source-build pitfalls section is real advice for a problem
we do not have. That removes the maintainability argument that was the artifact's
main reason for preferring Fortress, and the EOL argument against Fortress
(May 2027, the competition month) still stands.

### The package conflict, RQ-01's "conflict-resolution procedure"

There is a conflict, and it is a plain Debian `Conflicts:`, not a procedure:

```text
ros-humble-ros-gzharmonic         Conflicts: ros-humble-ros-gz, ros-humble-ros-gzgarden
ros-humble-ros-gzharmonic-bridge  Conflicts: ros-humble-ros-gz-bridge, ros-humble-ros-gzgarden-bridge
```

The flavours are mutually exclusive. Install one, never both. The Dockerfile
installs the Harmonic flavour only and never pulls `ros-humble-ros-gz`, so apt
never has to resolve anything.

### The one genuine source build, which we have NOT done

`gz_ros2_control` has **no Harmonic binary for Humble anywhere**:

- `ros-humble-gz-ros2-control` 0.7.20 on packages.ros.org depends on
  `libignition-gazebo6`, which is **Fortress**, despite the `gz-` name.
  `ros-humble-ign-ros2-control` is only a transitional alias for it.
- packages.osrfoundation.org ships no `gz_ros2_control` at all.

So on Humble + Harmonic, `gz_ros2_control` is a real source build. It is
deliberately absent from the image because **RQ-06 is not decided**: a
`grr_hardware/GazeboDriveHardware` topic bridge mirroring `IsaacDriveHardware`
would avoid it entirely and keep the two simulators architecturally identical.
Decide RQ-06 first. Do not add `gz_ros2_control` to the image by reflex.

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
though it has no ROS Humble of its own. This also means the "WSL has no Humble"
objection never applied to the GUI question.

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

Everything below was run on 2026-09-10 on the machine named at the top, through
the compose service, with the scripts executed straight off the bind mount.

```text
docker compose -f docker-compose.windows.yml run --rm igvc_gazebo \
  bash -lc "bash .../scripts/gazebo/render_check.sh"

  GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
  camera topic    : PUBLISHING
  gpu_lidar topic : PUBLISHING
  RESULT: HARDWARE RENDERING (NVIDIA)
```

```text
docker compose -f docker-compose.windows.yml run --rm igvc_gazebo \
  bash -lc "bash .../scripts/gazebo/bridge_smoke_test.sh"

  --- ROS 2 topics visible ---
  /clock
  /render_check/camera
  /render_check/scan

  camera  -> sensor_msgs/Image       PASS
  lidar   -> sensor_msgs/LaserScan   PASS
  clock   -> rosgraph_msgs/Clock     PASS
  passed: 3   failed: 0
```

So the whole chain is proven: Gazebo Harmonic renders on the discrete GPU, and
`ros_gz` delivers correctly typed sensor data and simulated time to ROS 2
Humble nodes. That is the part of the section 3.7 interface contract Gazebo can
satisfy without any of our own code. The ZED namespace, the joint-state pair and
TF are not covered and depend on RQ-03 and RQ-06.

---

## 5. What is in the image

Base `ros:humble-ros-base` (Ubuntu 22.04 jammy), plus:

- `gz-harmonic` and `ros-humble-ros-gzharmonic` from packages.osrfoundation.org
- `ros-humble-simulation-interfaces` 1.4.0, **message definitions only**. Per
  RQ-05 there is no Gazebo server on Harmonic that answers those services; the
  server implementation is Jetty-only. It is installed so
  `igvc_simulation_interface` still compiles, **not** because the services will
  work. Use `ros_gz_bridge` plus `gz service` for spawn and sim control.
- `ros-humble-xacro`, `ros-humble-robot-state-publisher`, `ros-humble-rviz2`
- `mesa-utils`, `vulkan-tools` for diagnosis

Image size 920 MB. It does **not** contain the workspace's Python/ML stack; it
is a simulator, not a replacement for `igvc-humble-fused-drive`.

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
