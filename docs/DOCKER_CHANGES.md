# Docker environment — change record

Documents every change made to the Docker setup in this fork, the reason for
each one, and how it was verified. Written so a reviewer can check the claims
rather than take them on trust.

- **Fork:** `wworth-IGVC/IGVC_robot_2027` (of `Gold-Rush-Robotics/IGVC_robot_2026`)
- **Baseline commit:** `a4b7433` (the fork point — nothing below this line existed then)
- **Tested on:** Windows 11 (build 26200), Docker Engine 29.7.2, Compose v5.5.1,
  WSL2 / Ubuntu 26.04, NVIDIA RTX 5070 Ti Laptop (Blackwell, sm_120)

## Commits

| Commit | Change | Covered in |
| --- | --- | --- |
| `e1b118d` | Fix the Humble image so the workspace builds (0/22 → 22/22) | §2, §3.1–3.3, §6.1 |
| `c5c88f0` | Add the buildable ZED image, Windows setup script, and these docs | §3.4, §4, §6.2, §6.4, §6.5 |
| `63a075a` | Update the README for the 2027 fork | §6.3 |
| `c6cc83d` | Close documentation gaps found by auditing against the diff | §6.3–6.5 |

Anything after those is described in the sections below; this table is not
maintained per-commit. For the authoritative list and the whole change set:

```bash
git log --oneline a4b7433..HEAD
git diff a4b7433..HEAD --stat
```

---

## 1. Summary

Three outcomes.

**1. The Humble image now builds.** `igvc_humble_fused_drive` could not build
**any** ROS package. A `colcon build` of the workspace produced:

```text
Summary: 0 packages finished
  1 package failed: debug_gui
  21 packages not processed
```

After the fixes in §2:

```text
Summary: 22 packages finished [4min 15s]
  2 packages had stderr output: odrive_ros2_control sllidar_ros2
COLCON_EXIT=0
```

The two remaining stderr packages emit compiler warnings only — see §5.3.

**2. The private ZED image has an open replacement.** `docker-compose.yml`
depended on `ghcr.io/gold-rush-robotics/dev-zed`, which is private, which most
team accounts cannot pull, and for which no build recipe is published anywhere
in the organisation (§5.2 — verified, not assumed).
`docker/Dockerfile.igvc-zed-humble` builds an equivalent x86 image from public
sources, and the full workspace builds inside it: **22 packages, exit 0** (§7.1).

**3. Windows works.** `docker-compose.yml` cannot run on Docker Desktop at all —
it bind-mounts `/dev`, sets `network_mode: host`, and requests
`runtime: nvidia`. `docker-compose.windows.yml` plus `scripts/setup-windows.ps1`
give the three Windows machines on the team a working path (§3.3, §6.2).

Everything claimed here was verified against real build output. §7 lists each
check and the command that produces it. Where an early diagnosis turned out to
be wrong, the correction is recorded rather than quietly dropped — see §4.2.1.

---

## 2. Image fixes (`docker/Dockerfile.humble-fused-drive`)

### 2.1 setuptools / packaging conflict — broke all 22 packages

**Symptom.** Every `ament_python` package failed:

```text
TypeError: canonicalize_version() got an unexpected keyword argument
           'strip_trailing_zero'
```

**Root cause.** The original single `RUN` layer ended with:

```dockerfile
&& pip3 install --no-cache-dir -U pip \
&& pip3 install --no-cache-dir 'numpy<2' ultralytics opencv-python-headless torch
```

That cascade installed **setuptools 84.0.0**. `colcon-core 0.21.1` requires
`setuptools<80`, and pip said so during the build:

```text
ERROR: pip's dependency resolver does not currently take into account all the
packages that are installed... colcon-core 0.21.1 requires setuptools<80,>=30.3.0,
but you have setuptools 84.0.0 which is incompatible.
```

setuptools 84 calls `canonicalize_version(version, strip_trailing_zero=False)`,
but the `packaging` library present in the image predates that keyword.

**Why it hid everything else.** `colcon build` aborts on the first failure
unless `--continue-on-error` is passed. `debug_gui` sorts first alphabetically,
so its failure stopped the run and the other 21 packages reported as
`not processed` — not as passing. The single defect masked the whole picture.

**Fix.** Pin setuptools in its own layer, *after* the torch install, since that
install is what drags it forward:

```dockerfile
RUN pip3 install --no-cache-dir 'setuptools==58.2.0' 'packaging<25'
```

Resolves to setuptools 58.2.0 / packaging 21.3.

**Verified.** `debug_gui` went from a hard crash to `Finished <<< debug_gui
[0.70s]`, and the run advanced from 0/22 to 20/22.

### 2.2 Missing `libasio-dev` — broke `ublox_gps` and `ublox`

**Symptom.**

```text
CMake Error at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:230
  Could NOT find asio (missing: ASIO_INCLUDE_DIR)
Call Stack:
  cmake/Findasio.cmake:4 (find_package_handle_standard_args)
  CMakeLists.txt:16 (find_package)
```

**Root cause.** `src/ros2-ublox-zedf9p/ublox_gps/package.xml` declares
`<depend>asio</depend>`, which rosdep resolves to `libasio-dev`. The Dockerfile
installed its dependencies as a hand-maintained apt list that omitted it. The
`ublox` metapackage depends on `ublox_gps`, so it was blocked too.

**Fix.** Added `libasio-dev` to the apt list.

**Verified.** `Finished <<< ublox_gps [40.9s]`, `Finished <<< ublox [0.68s]`.

### 2.3 Missing `diagnostic_updater`

`ublox_gps` also declares `<depend>diagnostic_updater</depend>`, absent from the
image. This was hidden behind the asio failure — CMake stopped at the first
missing dependency. Added `ros-humble-diagnostic-updater`.

### 2.4 Missing `python3-pyqt5` and `sensor_msgs_py`

`src/debug_gui/package.xml` declares `<exec_depend>python3-pyqt5</exec_depend>`.
Being an exec dependency, the package *builds* without it and fails only when
run — a latent runtime failure rather than a build error. Added
`python3-pyqt5` and `ros-humble-sensor-msgs-py`.

### 2.5 Duplicate OpenCV installation

The pip line requested `opencv-python-headless`, but `ultralytics` pulls in the
full `opencv-python` as a transitive dependency, so both ended up installed. Two
OpenCV builds in one environment is a known source of Qt/GUI conflicts in
headless containers. Now:

```dockerfile
&& pip3 uninstall -y opencv-python || true
```

Image size went from 4.04 GB to 3.98 GB as a side effect.

### 2.6 Structural change

The original Dockerfile did apt and pip work in one `RUN`. It is now split into
apt / pip / setuptools-pin layers. This is what makes the setuptools pin land
last, and it means changing the Python stack no longer invalidates the apt layer
on rebuild.

---

## 3. Compose changes

### 3.1 `docker-compose.yml` — restored the Humble service

The entire `igvc_humble_fused_drive` service (previously lines 37–90) was
commented out, while `README.md` documented it as the supported workflow:

```bash
docker compose build igvc_humble_fused_drive   # failed: no such service
```

The service is now uncommented and functional.

### 3.2 `DEVENV_HOST_PATH` no longer hardcoded

It was committed as:

```yaml
- DEVENV_HOST_PATH=/home/nitin-5090/Documents/DevEnv/jazzy_ws/IGVC_robot_2026
```

That is another developer's home directory. The variable is **not** dead — it is
read by `src/igvc_simulation_interface/igvc_simulation_interface/simulation_interface.py:95`
to map container paths to host paths, and the code warns once if unset. So it
was parameterized rather than deleted:

```yaml
- DEVENV_HOST_PATH=${DEVENV_HOST_PATH:-}
```

The path also explains its own origin: `Gold-Rush-Robotics/DevEnv` is a public
repo whose `.devcontainer/devcontainer.json` sets
`"DEVENV_HOST_PATH": "${localWorkspaceFolder}"`. The intended workflow is to
place this repo inside `DevEnv/jazzy_ws/`. The parameterized form is compatible
with that — the devcontainer supplies the value.

### 3.3 `docker-compose.windows.yml` — new

`docker-compose.yml` cannot run on Docker Desktop for Windows. It bind-mounts
`/dev`, `/tmp`, and `/tmp/.X11-unix`, sets `network_mode: host`, and requests
`runtime: nvidia` — none of which are valid there.

This is a **separate file**, not an override, because Compose merges volume
lists by target and therefore cannot *remove* an inherited mount. It must be
passed explicitly:

```bash
docker compose -f docker-compose.windows.yml build igvc_humble_fused_drive
```

Name the service explicitly. Since §4.5.4 added the two ZED variants, a bare
`docker compose build` builds all three images — about 40 minutes and 70 GB.

GPU access is preserved through `gpus: all`. Build output goes to named volumes
so colcon artifacts stay off the (slow) Windows bind mount.

### 3.4 `igvc_zed_humble` service — new, in both compose files

Service for the consolidated ZED image described in §4. It follows the same
shape as `igvc_humble_fused_drive`: repo bind-mounted at
`/root/ros2_ws/src/IGVC_robot_2026`, with `igvc_zed_build` / `igvc_zed_install` /
`igvc_zed_log` named volumes for the colcon output directories.

**One subtlety, checked rather than assumed.** Unlike the fused-drive image, this
image ships a populated `/root/ros2_ws/install` — that is where the six ZED
packages live. Mounting a named volume over it looks like it would hide them.
It does not: Docker seeds an empty *named* volume from the image's content at
that path on first creation. Verified:

```text
$ docker run --rm -v zedseedtest_install:/root/ros2_ws/install \
    igvc-zed-humble:latest bash -lc 'ros2 pkg list | grep -ci zed'
6
```

The seeding happens **only when the volume is first created**. If the image is
later rebuilt with different ZED content, an existing `igvc_zed_install` volume
will keep serving the old copy. After rebuilding the image, run:

```bash
docker compose down -v
```

A bind mount at the same path would have hidden the packages outright — this
works only because these are named volumes.

---

## 4. New: consolidated ZED image (`docker/Dockerfile.igvc-zed-humble`)

### 4.1 Problem

`docker-compose.yml`'s primary service uses
`ghcr.io/gold-rush-robotics/dev-zed:5.1.0-13.0.0`, which is a **private** GHCR
package:

```text
$ docker pull ghcr.io/gold-rush-robotics/dev-zed:5.1.0-13.0.0
Error response from daemon: error from registry: unauthorized
```

Forking the repository does not grant registry access. A search of every public
Gold-Rush-Robotics repository (§5.2) found **no build recipe for it anywhere**,
so it cannot be reproduced either — it is an unreproducible binary dependency.

### 4.2 Approach

Build an equivalent from public sources, on Humble rather than Jazzy. Contents:

| Component | Version | Why |
| --- | --- | --- |
| Ubuntu | 22.04 (Jammy) | Required by ROS 2 Humble |
| CUDA | 13.0 | Blackwell (RTX 50-series) needs ≥ 12.8 |
| ZED SDK | 5.3.0 | Wrapper requires ≥ 5.2; matches `jetson-zed` |
| ROS 2 | Humble | Matches the Jetson images |
| ZED ROS 2 | wrapper + `zed_msgs` + `zed_description` | `zed_components` needs all three |
| Plus | Nav2, ros2_control, PyTorch | Merges in the fused-drive image |

The SDK URL was verified to serve a real binary before relying on it:

```text
$ curl -sIL https://download.stereolabs.com/zedsdk/5.1.0/cu13/ubuntu22
Location: .../ZED_SDK_Ubuntu22_cuda13.0_tensorrt10.13_v5.1.0.zstd.run
Content-Type: binary/octet-stream
Content-Length: 1630536643
```

`nvidia/cuda:13.0.0-devel-ubuntu22.04` was likewise confirmed to exist — CUDA 13
does still publish Ubuntu 22.04 images, which is what makes a CUDA-13 + Humble
combination possible at all.

### 4.2.1 How the SDK version was chosen — including one wrong answer

This section records the sequence honestly, because the middle step **looked
verified and was not**. If you only read one thing here, read §4.5.

| Attempt | SDK | Outcome |
| --- | --- | --- |
| 1 | 5.1.0 | fails to compile |
| 2 | 5.3.0 | compiles, all checks pass, **node dies on launch** |
| 3 | 5.2.3 | works — verified by launching the node (§7.2) |

The first build attempt pinned SDK **5.1.0**, matching the version in the old
`dev-zed:5.1.0-13.0.0` tag. `zed_components` failed to compile:

```text
error: 'struct sl::CustomObjectDetectionProperties' has no member
       named 'object_tracking_parameters'
```

Repeated across ~18 call sites in `zed_camera_component_objdet.cpp`.

This is an SDK **API mismatch**, not a ROS distro problem. The wrapper's own
README states it requires *"ZED SDK v5.2"*; `object_tracking_parameters` was
introduced in 5.2, so the current wrapper source cannot build against 5.1.0.

Attempt 2 pinned **5.3.0**, reasoning that it satisfies the `>= 5.2`
requirement and matches `docker_images/jetson-zed`, so x86 and Jetson would run
the same SDK.

That build was then declared "confirmed fixed" on this evidence:

```text
$ grep PACKAGE_VERSION /usr/local/zed/zed-config-version.cmake
set(PACKAGE_VERSION "5.3.0")

$ ros2 pkg list | grep -i zed
zed_components  zed_debug  zed_description
zed_msgs        zed_ros2   zed_wrapper
```

plus `zed_components` compiling in 1 min 29 s, `Summary: 6 packages finished`,
and the 22-package IGVC workspace building inside the image with exit 0.

**All of that was true, and the image was still broken.** Launching the node
gives:

```text
[ERROR] This version of the ZED ROS2 wrapper is designed to work with
        ZED SDK v4.2 or newer up to v5.2.
[INFO]  * Detected SDK v5.3.0 ... Node stopped.
```

The reasoning error was assuming a *minimum* requirement when the wrapper
enforces a **closed range** — and the upper bound is only checked at runtime.
The lesson generalises past this repo: *a build is not a test of a program that
validates its environment when it starts.*

Attempt 3 pinned **5.2.3**, the newest patch inside the org fork's supported
window, and verified it by launching the node (§7.2).

The full analysis — why the cap exists, why `jetson-zed` is unaffected, and the
two supported wrapper/SDK pairings now provided — is in **§4.5**.

### 4.3 Consolidation

This merges what were two x86 images into one, so the x86 side needs one image
instead of two, and no private one:

| Before | After |
| --- | --- |
| `dev-zed` (private, Jazzy, ZED SDK) | `igvc-zed-humble` |
| `igvc-humble-fused-drive` (Humble, Nav2, torch) | ↑ same image |

`BASE_IMAGE`, `ZED_SDK_*`, and `CUDA_MAJOR` are build args so the same recipe
can target a different CUDA level or an L4T base for the Jetson.

**A single image for every machine is not achievable**, for two hard reasons:
CPU architecture (Jetson is arm64/L4T, laptops are x86_64) and Ubuntu version
(Humble needs 22.04, Jazzy needs 24.04). One image per architecture, from one
Dockerfile, is the realistic floor.

### 4.4 Which image to use

Both are kept; neither replaces the other.

| Task | Image | Download size | On disk |
| --- | --- | --- | --- |
| Everyday development | `igvc_humble_fused_drive` | 3.98 GB | **13 GB** |
| ZED camera code, robot machines | `igvc_zed_humble` | 9.70 GB | **28.6 GB** |

**Two numbers, because Docker reports two and they differ by ~3.3×.** Quote the
right one:

```text
$ docker image inspect igvc-zed-humble:latest --format "{{.Size}}"
9695080432                 # 9.70 GB - sum of COMPRESSED layers = download size

$ docker image ls igvc-zed-humble
igvc-zed-humble  latest  28.6GB      # unpacked, once actually used
```

This machine uses the containerd image store
(`driver-type: io.containerd.snapshotter.v1`, overlayfs), which unpacks layers
into snapshots **lazily — on the first container run, not at build time**. That
is measurable, and it is why a freshly built image can look deceptively small:

```text
# immediately after `docker compose build igvc_zed_humble_upstream`
igvc-zed-humble  upstream   ls=9.7GB

# after one `docker run` from that same image
igvc-zed-humble  upstream   ls=28.6GB

# inspect is unchanged throughout: 9.70 GB
```

So **an image you have built but not yet run has not finished costing you
disk.** Budget the larger number, and add build cache on top — it reached
**55.9 GB** while developing these images. `docker system df` shows all of it;
`docker builder prune` reclaims the cache.

The download size still came in well under the 15–20 GB originally projected,
because `skip_cuda` and `skip_tools` keep the SDK installer from duplicating
CUDA or adding the GUI tools.

On Windows the smaller image is the right default: Docker Desktop cannot pass
through USB, so a ZED camera is unusable there, and the 22 workspace packages
build without the SDK (proven — the 4 GB image has no SDK and builds 22/22).

### 4.5 Wrapper / SDK pairing, and the two variants

This is the most important section in this document for anyone maintaining the
ZED image. **A build-only test cannot validate this image.**

#### 4.5.1 The wrapper enforces a closed SDK range, at runtime

`zed_components/src/include/sl_version.hpp` defines a minimum *and* a maximum
supported SDK, checked in `zed_camera_component_main.cpp` as
`(MAJOR * 10 + MINOR)` — the patch level is not considered:

```cpp
if (((ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) <
  (SDK_MAJOR_MIN_SUPP * 10 + SDK_MINOR_MIN_SUPP)) ||
  ((ZED_SDK_MAJOR_VERSION * 10 + ZED_SDK_MINOR_VERSION) >
  (SDK_MAJOR_MAX_SUPP * 10 + SDK_MINOR_MAX_SUPP)))
```

Both ends fail, but they fail in **very different ways**:

| SDK | Wrapper | Result |
| --- | --- | --- |
| 5.1.0 | org fork (max 5.2) | fails to **compile** — obvious, caught by any build |
| 5.3.0 | org fork (max 5.2) | **compiles perfectly, node refuses to start** |
| 5.2.3 | org fork (max 5.2) | works |
| 5.3.0 | upstream v5.4.1 (max 5.4) | works |

The 5.3.0 + fork combination is the dangerous one. The image builds with no
errors, all six ZED packages install, `ros2 pkg list` shows them — and then:

```text
[INFO] Load Library: .../libzed_camera_component.so
[INFO] Found class: rclcpp_components::NodeFactoryTemplate<stereolabs::ZedCamera>
[INFO] ================================
[INFO]       ZED Camera Component
[ERROR] This version of the ZED ROS2 wrapper is designed to work with
        ZED SDK v4.2 or newer up to v5.2.
[INFO] * Detected SDK v5.3.0-114247_4d466e77_3096846
[INFO] Node stopped. Press Ctrl+C to exit.
[ERROR] process has died [exit code 1]
```

**This image was originally published in exactly that broken state**, because
it had been verified by building it and by running `colcon build` inside it,
never by launching the node. If you change `ZED_WRAPPER_BRANCH` or any
`ZED_SDK_*` value, verify with §7.2, not with a build.

#### 4.5.2 Why the cap exists: the org fork is a stale mirror

The cap is not upstream's. Upstream derives its maximum from its own version:

```cpp
const size_t SDK_MAJOR_MAX_SUPP = WRAPPER_MAJOR;   // 5
const size_t SDK_MINOR_MAX_SUPP = WRAPPER_MINOR;   // 4
```

`Gold-Rush-Robotics/zed-ros2-wrapper-2025` is pinned at wrapper **v5.2.1**,
which is where the hard 5.2 cap comes from. Comparing it against upstream:

```text
GET /repos/stereolabs/zed-ros2-wrapper/compare/
      master...Gold-Rush-Robotics:zed-ros2-wrapper-2025:master

status    : behind
ahead_by  : 0        <- commits the fork has that upstream does not
behind_by : 70       <- commits upstream has that the fork lacks
files changed vs upstream: 0
```

**The fork contains no org-specific changes at all.** It is a plain mirror,
70 commits stale. There is nothing in it to preserve.

#### 4.5.3 `jetson-zed` is not affected

Worth stating plainly, because the mismatch above raised the question.
`docker_images/jetson-zed` pairs SDK 5.3.0 with **upstream**
(`git clone https://github.com/stereolabs/zed-ros2-wrapper.git`), whose cap is
5.4. `53 <= 54`, so it is internally consistent. No latent fault there.

One real weakness though: it clones upstream with **no tag or branch**, so its
result depends on when it was built. Both variants below pin their wrapper.

#### 4.5.4 The two variants

Which one the team adopts is an open decision, so both are wired up rather than
one being chosen unilaterally.

| | `igvc_zed_humble` (A) | `igvc_zed_humble_upstream` (B) |
| --- | --- | --- |
| Wrapper | org fork v5.2.1 | upstream **v5.4.1**, pinned |
| ZED SDK | 5.2.3 | 5.3.0 |
| Image tag | `igvc-zed-humble:latest` | `igvc-zed-humble:upstream` |
| Matches `jetson-zed` | no | **yes** — same wrapper source and SDK |
| Wrapper currency | 70 commits behind upstream | current |

```bash
docker compose build igvc_zed_humble            # A
docker compose build igvc_zed_humble_upstream   # B
```

They use separate colcon volumes (`igvc_zed_*` vs `igvc_zed_up_*`), so both can
exist side by side without mixing build output.

**The argument for B**: it is the same wrapper source and SDK the Jetson
already runs, which is what "consolidation" was supposed to achieve, and it
picks up 70 commits of upstream fixes. The fork it would replace contributes
nothing (§4.5.2).

**The argument for A**: it is the combination the repo implicitly declared by
referencing the org fork, and it changes less. If someone forked
`zed-ros2-wrapper-2025` intending to customise it later, A keeps that path open.

A is the default only because it is the more conservative of the two, not
because it is better.

---

## 5. Investigation findings

Recorded because they answer questions that will come up again.

### 5.1 Container registry inventory

Anonymous pull test against `ghcr.io/gold-rush-robotics/*`:

| Image | Result |
| --- | --- |
| `dev_env` | HTTP 200 — public |
| `jetson-zed` | HTTP 200 — public |
| `dev-zed` | HTTP 403 — private |
| `jetson-ros-base` | HTTP 403 — private |
| `isaac-ros` | HTTP 403 — private |

Available tags: `dev_env` → 1–7, 9, latest. `jetson-zed` → 5.0-36.4,
5.1-36.4.7, 5.3-36.4.7.

### 5.2 There is no public `dev-zed` recipe

Ten public org repositories were downloaded and searched. `dev-zed` / `dev_zed`
returned **zero matches**. Every `ghcr.io/gold-rush-robotics` reference in all
of them points to only three images: `jetson-ros-base` (5), `dev_env` (2),
`rmw-zenoh` (1).

`docker_images/jetson-zed` is a **different image**, not a substitute: its
manifest reports `"architecture": "arm64"` and it is built `FROM
nvcr.io/nvidia/l4t-jetpack:r36.4.0`. It cannot run on an x86 laptop.

### 5.3 Not defects — checked and deliberately left alone

- **`models/` missing.** Intentional. It is gitignored, and
  `src/igvc_lane_detection/scripts/fetch_yolopv2_weights.sh` populates it. A
  setup step, not a bug.
- **`simulation_interfaces` in `--packages-select`.** It is the apt-provided
  `ros-humble-simulation-interfaces`, not a workspace package. colcon logs
  `ignoring unknown package 'simulation_interfaces'` and still exits 0.
- **`odrive_ros2_control` / `sllidar_ros2` stderr.** Vendor compiler warnings
  (`-Wunused-parameter`, zero-size arrays under `-Wpedantic`). Not errors.
- **`/root/ros2_ws/src/IGVC_robot_2026` mount path.** Still says 2026 in a fork
  named 2027, which looks like an oversight. It is not: the path is the
  container-side mount target, referenced by `docker-compose.yml`,
  `IGVC_WORKSPACE_ROOT`, `YOLOPV2_WEIGHTS`, and the DDS profile paths. Renaming
  it would break all of them for no benefit.
- **Hard tabs in a README bash code block.** Pre-existing at `a4b7433`. Flagged
  by markdownlint (MD010); tabs are valid in shell, so this is cosmetic and was
  left to keep the diff focused.
- **Stray `# ros2 launch ... isaac_nav_test` comment** in the `volumes:` block of
  `docker-compose.yml`. Looks like it was displaced by an earlier edit, but it is
  pre-existing at `a4b7433` and harmless.

### 5.4 Humble / Jazzy audit

The org's images are genuinely mixed:

- **Humble:** `jetson-zed`, `jetson-ros-base`, `jetson-isaac-ros`, `isaac-ros`,
  `humble-rust`
- **Jazzy:** `dev_env`, `circuit-python-ros`, `kria`, `oakd-ros`, `zenoh`

Everything on the **robot side is already Humble**. The Jazzy images are dev
tooling and peripherals. The only Jazzy component in the robot path was the
private `dev-zed`, which §4 replaces.

**Mislabeled tag.** `docker_images/jetson-ros-base/build` sets
`VERSION="jazzy-36.4.7-2"`, but its Dockerfile builds **Humble**
(`ARG ROS2_DIST=humble`, `ENV ROS_DISTRO=humble`, installs `ros-humble-ros-core`
and `ros-humble-ros-base`, runs rosdep `--rosdistro humble`). The image tagged
`jazzy-...` contains Humble. Worth correcting upstream — it is actively
misleading.

**Stale comments in this repo.** `docker-compose.jetson.yml` describes
`igvc_jetson_zed` as *"ROS 2 Jazzy + ZED SDK 5"* and *"Includes: … ROS 2
Jazzy"*, but the image is Humble and the service's own commented command sources
`/opt/ros/humble/setup.bash`. Comments only — no functional effect. Left for a
separate change.

**Submodules point upstream, not at the org forks.** `isaac/exts` tracks
`stereolabs/zed-isaac-sim` and `src/zed-description` tracks
`stereolabs/zed-ros2-description`, although the org maintains its own
`zed-isaac-sim` and `zed-ros2-wrapper-2025` forks. May be deliberate; worth
confirming.

---

## 6. Supporting changes

### 6.1 `.gitattributes` — new

With `core.autocrlf=true` (the Windows default) and no `.gitattributes`, a
Windows clone rewrites shell scripts to CRLF. A CRLF script fails inside a Linux
container with:

```text
bad interpreter: /usr/bin/env bash^M
```

`fetch_yolopv2_weights.sh` is a documented setup step, so this would have hit
every Windows teammate. The file forces LF on scripts and configs and marks
binaries explicitly.

> **Note for existing clones:** this changes line endings at the next checkout.
> Commit or stash local work before pulling it.

### 6.2 `scripts/setup-windows.ps1` — new

Prerequisite checker and installer for Windows, written for teammates new to
Docker. Checks Windows build, WSL2, Docker daemon, Compose, GPU passthrough,
disk space, submodule population, and model weights; explains what to do about
each failure; then builds. `-CheckOnly` diagnoses without changing anything.

Written for Windows PowerShell 5.1 (what Windows 10 ships), so it avoids
ternary, null-coalescing, and `&&` chaining.

Two bugs found while testing it, both worth knowing independently:

1. **`$?` after a redirected native command.** In PowerShell 5.1,
   `(docker ... 2>&1) | Out-String` sets `$?` to `$false` even on exit code 0,
   because each stderr line becomes an ErrorRecord. Use `$LASTEXITCODE`.
2. **`nvidia-smi` output differs under WSL2.** It prints
   `NVIDIA-SMI x  KMD Version: y  CUDA UMD Version: z` — the string
   `Driver Version` never appears. Matching on `Driver Version` false-alarms on
   every Windows machine. The script matches `NVIDIA-SMI` instead.

### 6.3 `README.md`

**Windows support (in `e1b118d`).** New section covering WSL2 (Option A) vs
PowerShell (Option B), the setup script, GPU/Blackwell notes, and ZED
limitations on Windows.

**Fork identity and stale content (in `63a075a`).**

- Retitled from `IGVC_robot_2026` to `IGVC_robot_2027`, with a link crediting
  the upstream repo.
- Added a "What changed in this fork" section so a teammate landing on the repo
  sees the state without reading this file first.
- **Runtime targets** table listed only Jazzy-on-host and Humble-in-Docker. It
  now lists both Humble images with their measured sizes and demotes Jazzy to
  host-side tooling, matching the goal of consolidating the robot side on
  Humble.
- **Repository layout** table did not mention `docker-compose.windows.yml` or
  `scripts/`; both are now listed.
- **Maintainer notes** now say to mirror any new compose service into the
  Windows file, and to pin `setuptools` after touching a Dockerfile pip stack —
  the exact trap in §2.1.

#### 6.3.1 A broken command in the clean-rebuild instructions

**Symptom.** The documented way to force a clean workspace rebuild fails:

```text
$ docker volume rm igvc_robot_2026_igvc_humble_build ...
Error response from daemon: get igvc_robot_2026_igvc_humble_build: no such volume
```

**Root cause.** Compose derives volume names from the **checkout directory**, so
in a fork checked out as `IGVC_robot_2027` the volumes are
`igvc_robot_2027_igvc_humble_*`. The README hardcoded the upstream 2026 names.

A note explaining exactly this already existed in the README — but 77 lines
further down, past the point where anyone copy-pasting the command would already
have hit the error.

**Fix.** Replaced the hand-written volume names with a command that does not
depend on the directory name at all, and moved the explanation next to it:

```bash
docker compose down -v
```

The same edit adds a point that was missing everywhere: `down -v` is also
required **after rebuilding an image**, because named volumes are seeded from
the image only on first creation (§3.4). Without it, a rebuilt image silently
keeps serving the previous build output.

### 6.4 `.gitignore` — Office lock files

Opening `docs/IGVC_2027_Docker_Setup_Guide.docx` in Word creates a hidden
owner file, `~$VC_2027_Docker_Setup_Guide.docx`, in the same directory. It
showed up as untracked and would have been committed by a `git add -A`. Added:

```gitignore
~$*
```

Word also holds an exclusive lock on the open file, so regenerating the guide
while it is open fails with `PermissionError`. The generator now honours a
`DOCX_OUT` environment variable so it can write elsewhere and be copied in
afterwards.

### 6.5 Documentation artifacts

- `docs/IGVC_2027_Docker_Setup_Guide.docx` — step-by-step guide for teammates
  who have never used Docker. Sizes and SDK versions in it are the measured
  ones from §7, not estimates.
- `docs/DOCKER_CHANGES.md` — this file.

---

## 7. Verification log

| Check | Command | Result |
| --- | --- | --- |
| Image builds clean | `docker build -f docker/Dockerfile.humble-fused-drive .` | 3.98 GB, exit 0 |
| Workspace builds | `colcon build` (22 packages) | `22 packages finished`, `COLCON_EXIT=0` |
| GPU passthrough | `docker run --gpus all nvidia/cuda:… nvidia-smi` | RTX 5070 Ti visible |
| Blackwell support | `torch.cuda.get_arch_list()` | includes `sm_120`; device capability `(12, 0)` |
| Compose validity | `docker compose config --quiet` (both files) | valid |
| Compose package-select | exact `--packages-select` list from the service | 6 packages, exit 0 |
| Setup script syntax | PowerShell AST parse | no errors under 5.1 |

### 7.1 `igvc-zed-humble` (consolidated ZED image)

| Check | Command | Result |
| --- | --- | --- |
| Image builds clean | `docker build -f docker/Dockerfile.igvc-zed-humble .` | exit 0, **18.5 min**, 9.70 GB download / **28.6 GB on disk** |
| ZED SDK installed | `grep PACKAGE_VERSION /usr/local/zed/zed-config-version.cmake` | `5.2.3` |
| ZED wrapper compiles | colcon, layer 9/14 | `zed_components` in 1 min 29 s; `Summary: 6 packages finished` |
| ZED packages resolvable | `ros2 pkg list \| grep -i zed` | 6: `zed_components`, `zed_debug`, `zed_description`, `zed_msgs`, `zed_ros2`, `zed_wrapper` |
| **IGVC workspace regression** | `colcon build` over all repo packages, inside this image | **`22 packages finished [4min 22s]`, `COLCON_EXIT=0`** |
| Named volume does not hide ZED | empty named volume mounted over `/root/ros2_ws/install` | 6 packages still visible (§3.4) |
| Compose service parses | `docker compose config --services` (both files) | `igvc_zed_humble` present |

Layer timings, for anyone deciding whether to build or pull (see §8, item 2):

| Layer | Time |
| --- | --- |
| ZED SDK download + install | 217.9 s |
| pip / torch / ultralytics | 185.8 s |
| ROS 2 Humble apt | 72.6 s |
| rosdep install | 68.2 s |
| ZED wrapper colcon build | 122.4 s |
| IGVC apt dependencies | 66.7 s |
| Image export | 288.0 s |

Both workspace builds were verified against images built **from scratch** with
the corrected Dockerfiles, not against incrementally patched layers.

> **Every check in the table above also passed on the broken SDK 5.3.0 build.**
> Image built, six ZED packages resolved, 22 IGVC packages compiled — and the
> node still died on launch (§4.5.1). Build-time checks are necessary and not
> sufficient. §7.2 is the one that catches it.

### 7.2 Runtime check — the one that actually matters

Static checks cannot detect the wrapper's SDK guard, because it runs when the
node starts. This launches the node with the repo's own parameter override file
and no camera attached:

```bash
docker run --rm --gpus all \
  -v "<repo>:/root/ros2_ws/src/IGVC_robot_2026" \
  igvc-zed-humble:latest bash -lc '
    source /opt/ros/humble/setup.bash
    source /root/ros2_ws/install/setup.bash
    timeout 45 ros2 launch zed_wrapper zed_camera.launch.py \
      camera_model:=zed2i camera_name:=front_zed_camera_x \
      ros_params_override_path:=/root/ros2_ws/src/IGVC_robot_2026/src/igvc_test_bringup/config/common_stereo_real.yaml'
```

`--gpus all` is required. Without it the component fails earlier with
`libcuda.so.1: cannot open shared object file`, which is the test harness
missing the GPU, not an image fault.

**Interpreting the result.** There is no camera, so it must fail eventually.
What matters is *where*:

| Where it stops | Meaning |
| --- | --- |
| `designed to work with ZED SDK ... up to vX.Y` | **wrong SDK/wrapper pairing** (§4.5) |
| a parameter error | the override YAML is incompatible with this wrapper |
| `CAMERA STREAM FAILED TO START`, retrying | **pass** — software stack is fine, only hardware is absent |

Result for variant A (org fork wrapper + SDK 5.2.3):

```text
[INFO]  * Advertised on service: '/front_zed_camera_x/zed_node/enable_obj_det'
        ... (all services advertised)
[INFO] === STARTING CAMERA ===
[INFO] ZED SDK Version: 5.2.3 - Build 112767_50a3dc68_3019834
[INFO] === CAMERA OPENING ===
[WARN] Error opening camera: CAMERA STREAM FAILED TO START
[INFO] Please verify the camera connection
```

Passed: no version rejection, all services advertised, reached camera open and
retried cleanly. It also confirms the repo's `common_stereo_real.yaml` is
accepted — see §7.3.

Result for variant B (upstream wrapper v5.4.1 + SDK 5.3.0):

```text
[INFO] === STARTING CAMERA ===
[INFO] ZED SDK Version: 5.3.0 - Build 114247_4d466e77_3096846
[INFO] === CAMERA OPENING ===
[WARN] Error opening camera: CAMERA STREAM FAILED TO START
[INFO] Please verify the camera connection
```

Also passed. Note this is the **same SDK 5.3.0 that fails with the org fork** —
proof that the fault was never the SDK version on its own, only its pairing
with a wrapper that caps at 5.2.

**Both variants are verified.** Summary:

| Variant | Wrapper | SDK | Builds | Node launches |
| --- | --- | --- | --- | --- |
| `igvc_zed_humble` | org fork v5.2.1 | 5.2.3 | yes | **yes** |
| `igvc_zed_humble_upstream` | upstream v5.4.1 | 5.3.0 | yes | **yes** |
| *(rejected)* | org fork v5.2.1 | 5.3.0 | yes | **no** |

What cannot be verified on Windows: that a camera actually streams. Docker
Desktop cannot pass USB through, so the launch test above is the furthest any
check can go on a Windows machine. First real camera bring-up has to happen on
the Jetson or a native Linux host.

### 7.3 Repo/wrapper interface checks

Confirming the repo can actually drive this wrapper, not merely that the
wrapper exists:

| Check | Result |
| --- | --- |
| `zed_wrapper/launch/zed_camera.launch.py` present at the path `zed_multi.launch.py` resolves | yes |
| Launch arguments the repo passes vs those the wrapper declares | **14 of 14 accepted** (repo passes 14, wrapper declares 29) |
| Repo's `common_stereo_real.yaml` loaded | yes — `Using ROS parameters override file: ...` |
| ZED URDF parses | yes — all 6 camera frames published by `robot_state_publisher` |
| Parameters set by the repo but not declared by the wrapper | 29, mostly `object_detection.*`. **Silently ignored, not fatal** — the node started with the override applied. They have no effect, so object-detection tuning in that file is currently inert. |

The launch-argument comparison is done by AST-parsing `zed_multi.launch.py`
rather than by regex: the repo builds the dict and then adds three keys
conditionally, and a regex over the literal misses those and reports a false
pass.

---

## 8. Outstanding

1. **`dev-zed` access.** Still private. Either have the package made public /
   granted to team accounts, or adopt `igvc-zed-humble` (§4) and retire it.
2. **Publish `igvc-zed-humble`.** Building it per-developer costs ~18.5 minutes
   and **28.6 GB of disk** (plus build cache, which reached 55.9 GB here).
   Pulling a published image transfers 9.70 GB and skips the build entirely.
   Push it to GHCR once and have everyone pull.
3. **Correct the `jetson-ros-base` tag** upstream — `jazzy-36.4.7-2` contains
   Humble (§5.4).
4. **Fix the stale Jazzy comments** in `docker-compose.jetson.yml` (§5.4).
5. **Confirm submodule remotes** — upstream Stereolabs vs the org forks (§5.4).
6. **ZED camera on Windows.** `usbipd-win` can attach USB devices into WSL2, but
   ZED cameras are high-bandwidth USB3 and USB/IP handles that poorly. Test
   before relying on it.
7. **Run `render_check.sh` on the other two machines** (§10.2). The RTX 5080
   Laptop and the Windows 10 machine are untested. The Intel-adapter crash
   depends on which GPUs a laptop has, so results will differ per machine.
8. **Confirm WSLg on the Windows 10 machine** (§10.3). It should work on 22H2
   with the Store version of WSL (`wsl --update`), but nobody has tried it. If
   it does not, that machine is headless-only and everything else still works.
9. ~~**`gz_ros2_control` source build**~~ **RESOLVED by section 11** — it ships
   as a prebuilt binary on Jazzy and is installed in the image. Whether to
   *use* it rather than a topic bridge is still RQ-06, still open.
10. **Decide whether the other images need the GPU rendering settings** (§10.2).
    They would render `rviz2` on `llvmpipe` today. Fine for RViz; the ZED images
    set their own `LD_LIBRARY_PATH` for the SDK, so that change needs care.
11. **Port `Dockerfile.humble-fused-drive` to Jazzy** (section 11.5). The jammy
    to noble jump moves the Python and ML stack with it, so this is not a
    one-line change. It is the everyday image and needs the most care.
12. **Port `Dockerfile.igvc-zed-humble` to Jazzy** (section 11.5). ZED SDK, CUDA
    and wrapper pinning move together; per section 4.5.1 the wrapper enforces
    its SDK range at runtime. The private image it replaces, `dev-zed`, is
    already Jazzy.
13. **Verify the repo's packages build on Jazzy.** The 2027 team built four of
    them under `osrf/ros:jazzy-desktop`; all 22 have not been tried.
14. **The robot-side Jetson images are still Humble** (section 11.6) and face
    the same May 2027 EOL. Not started, not in this repo, needs the team lead.

---

## 9. The 2026 competition branch, and what the images were missing for it

Found on 2026-09-10 by auditing the organisation's GitHub repositories rather
than the local checkout. It changes what these images have to be able to build.

### 9.1 This fork is based on `main`, and `main` is not the competition code

`Gold-Rush-Robotics/IGVC_robot_2026` has **15 branches**. This fork descends from
`main`, last touched 2026-05-31. The branch named
**`test_comp_changes_isaac_smi`** ("test competition changes, Isaac Sim") is
**15 commits ahead of `main` and 1 behind**, spanning 23 to 29 May 2026, the week
before the June competition. It was **never merged**.

61 files differ. The ones that matter to these images:

| Change | Effect on the image |
| --- | --- |
| `src/igvc_pointcloud_tools`, a new `ament_cmake` package | **needs PCL, which was absent** |
| `anchor3dlane_node.py`, `anchor3dlane_infer.py`, `ufldv2_infer.py` | none at build time; their deps install via opt-in `scripts/setup_*.sh` |
| `lane_segmentation.py` rewritten, +1923/-256 | none |
| `mission_planner.py`, three new costmap nodes | none |
| `localization.py` deleted, entry point removed | none |
| `<exec_depend>yolo_msgs</exec_depend>` added | already satisfied by the `yolo_ros` submodule |
| `isaac/exts` submodule repointed | see 9.3 |
| `docker-compose.yml` gains `ipc: host` / `pid: host` | see 9.4 |

### 9.2 PCL was the only real gap, and it is now fixed

`igvc_pointcloud_tools/CMakeLists.txt`:

```cmake
find_package(PCL REQUIRED COMPONENTS common filters registration)
```

`pointcloud_merger_node.cpp` includes `pcl/registration/icp.h`,
`pcl/filters/voxel_grid.h`, `pcl/common/transforms.h` and `pcl/point_types.h`.
It does a one-shot ICP/FPFH geometric calibration of the partially overlapping
ZED point clouds.

Verified absent from `igvc-humble-fused-drive:latest`: no `PCLConfig.cmake`
anywhere on the image, so `find_package(PCL REQUIRED)` fails outright.

Added `libpcl-dev` to **both** `docker/Dockerfile.humble-fused-drive` and
`docker/Dockerfile.igvc-zed-humble`. The package's own manifest declares
`<depend>libpcl-all-dev</depend>`, which rosdep maps to `libpcl-dev` on Jammy.

**Cost: 136 additional packages**, because `libpcl-dev` hard-depends on
`libvtk9-dev`, which pulls Qt5 development tools. That is unavoidable if the
headers are wanted; Ubuntu does not ship a headers-only split for the three PCL
components this package uses.

`ros-humble-rclcpp-components` is listed alongside it because the package
registers a composable node, but it was **already present** in
`ros:humble-ros-base` (16.0.19). Listing it is documentation, not a fix.

Nothing else was needed. The Anchor3DLane and UFLDv2 nodes pull `mmcv-full` and
friends through `scripts/setup_anchor3dlane.sh` and `scripts/setup_ufldv2.sh` at
runtime, deliberately, and baking those into the image would add gigabytes for
code that is opt-in.

### 9.3 The `isaac/exts` submodule points at the wrong remote on `main`

```text
main:                         375eddd1 -> stereolabs/zed-isaac-sim (upstream)
test_comp_changes_isaac_smi:  72ff9c21 -> Gold-Rush-Robotics/zed-isaac-sim (fork)
```

Unlike `zed-ros2-wrapper-2025`, which is an empty mirror (§4.5.2), the
organisation's `zed-isaac-sim` fork carries **real changes**: 2 commits ahead, 4
files modified. The substantive one is `72ff9c21`, **"added urdf support"**,
2026-03-02. It rewrites camera and IMU prim path resolution so the ZED extension
can find cameras inside a URDF-imported hierarchy:

```python
# upstream
left_path = "/base_link/" + base_camera_model + "/CameraLeft"

# org fork
prefix_match = re.match(r'^(.*?)(zed_camera_.+?)_camera_link$', prim_name)
left_path = f"/{prefix}{camera_base}_camera_center/{prefix}{camera_base}_left_camera_frame/CameraLeft"
```

That prefix pattern is exactly this repository's URDF: `left_`, `front_` and
`right_` on `zed_camera_x`. **The upstream extension cannot locate the cameras on
a robot spawned from our URDF.** The same commit pins the OmniGraph ABI
`TARGET_VERSION` from `(2,184,5)` down to `(2,184,2)`.

This is not an image change and has **not** been altered here, because
repointing a submodule is a decision about which baseline the 2027 fork tracks,
and that belongs to the team lead. It is recorded so nobody spends a day
debugging why the simulated cameras never appear.

### 9.4 `ipc: host` and `pid: host` for the ZED simulation path

The competition branch added both to `igvc_dev_zed` in `docker-compose.yml`.
`docker-compose.jetson.yml` already carried them.

The reason is the ZED SDK's simulation input. `common_stereo_sim.yaml` sets
`sim_address: '127.0.0.1'`, and the Stereolabs Isaac extension streams over
**IPC** rather than RTSP when source and consumer are on the same machine. A
container in its own IPC namespace cannot attach to that.

Added to `igvc_zed_humble` and `igvc_zed_humble_upstream` in
`docker-compose.yml`. **Not** added to `docker-compose.windows.yml`: under Docker
Desktop the "host" is the WSL2 VM, not Windows, so sharing the namespace cannot
reach an Isaac Sim running on the Windows side. That limitation is independent of
these two flags.

### 9.6 Proof the image change is sufficient, and the one code change it needs

`igvc_pointcloud_tools` was compiled from the competition branch inside the
rebuilt image. PCL resolved immediately; the build reached the source and failed
on something unrelated:

```text
error: passing 'const rclcpp::Clock' as 'this' argument discards qualifiers
  246 |     RCLCPP_WARN_THROTTLE(
note:   in call to 'rclcpp::Time rclcpp::Clock::now()'
```

The package targets **Jazzy**, where `rclcpp::Clock::now()` is `const`. Humble
declares it non-const, so calling it through the const method
`lookup_transform(...) const` is rejected. This is consistent with the
competition branch's `docker-compose.yml`, which sources
`/opt/ros/jazzy/setup.bash` because the private `dev-zed` image was Jazzy. Our
consolidation target is Humble, so adopting that branch means porting it.

For this package the port is one substitution across five call sites:

```bash
sed -i 's|\*this->get_clock()|*const_cast<rclcpp::Clock *>(this->get_clock().get())|g'   src/igvc_pointcloud_tools/src/pointcloud_merger_node.cpp
```

Only the site in the const method actually fails; the other four are harmless to
change and it keeps the file consistent. With that applied, in this image:

```text
Starting >>> igvc_pointcloud_tools
Finished <<< igvc_pointcloud_tools [28.2s]
Summary: 1 package finished
COLCON_EXIT=0
```

Producing `libpointcloud_merger_component.so` and the `pointcloud_merger_node`
executable, linked against PCL. **So `libpcl-dev` was the only image-level gap.**

The patch is recorded here rather than applied, because the file does not exist
in this fork; it arrives only if the team adopts that branch (RQ-25).

One caveat: `ldd` on the built executable reports a single unresolved library
when run without sourcing `install/setup.bash`. That is expected for a colcon
overlay and was not investigated further.

### 9.5 What this does not resolve

Which branch the 2027 fork should track is an open question and outranks
everything else in the simulation work. See `RESEARCH_QUESTIONS_AND_UNRESOLVED.md`
section 3.0 and RQ-25. The images are now able to build either baseline, which is
the part that could be settled without the team lead.

---

## 10. The Gazebo image, and the two Windows traps it exposed

Added 2026-09-15. Full detail in `GAZEBO_SETUP.md`; this section records what
changed in the Docker layer and why.

### 10.1 New image and service

| | |
| --- | --- |
| Dockerfile | `docker/Dockerfile.gazebo-harmonic` |
| Image | `igvc-gazebo-harmonic:latest`, 920 MB |
| Service | `igvc_gazebo`, in `docker-compose.windows.yml` only |
| Contents | Gazebo Harmonic (`gz-sim` 8.15.0), ROS 2 Humble, all six `ros_gz` packages, `simulation_interfaces` message definitions, `rviz2`, `xacro`, `mesa-utils`, `vulkan-tools` |

It is a simulator, not a replacement for `igvc-humble-fused-drive`: it carries
no PyTorch or ML stack.

**No source build is required.** `ros-humble-ros-gzharmonic` 0.244.12-3jammy and
its five siblings ship as prebuilt jammy debs from
`packages.osrfoundation.org`. The apparent conflict with the official
`ros-humble-ros-gz*` packages is a plain Debian `Conflicts:` between the
Fortress and Harmonic flavours: install one, never both. The Dockerfile installs
the Harmonic flavour only.

**`gz_ros2_control` is deliberately absent.** `ros-humble-gz-ros2-control`
0.7.20 on packages.ros.org depends on `libignition-gazebo6`, which is
**Fortress**, despite the `gz-` name; `ros-humble-ign-ros2-control` is only a
transitional alias for it. Installing it would drag a second Gazebo into the
image. A Harmonic build for Humble is packaged nowhere, so that package is the
one genuine source build, and it is on hold pending RQ-06.

### 10.2 Trap one: `--gpus all` does not give you OpenGL

The first run of this image rendered **entirely on the CPU while looking
completely healthy**: `gz sim` started, the world loaded, camera and lidar
topics published at plausible rates, and `GL_RENDERER` was
`llvmpipe (LLVM 15.0.7, 256 bits)`.

`--gpus all` provides CUDA. It does not provide OpenGL, and there is no native
NVIDIA GL driver inside a WSL2 container at all. GL is served by Mesa's `d3d12`
driver on top of `/dev/dxg`. Four things are required, and missing any one of
them yields software rendering with no error:

1. `NVIDIA_DRIVER_CAPABILITIES=all` (set in the Dockerfile). The runtime default
   is `utility,compute`, which excludes graphics.
2. `--device=/dev/dxg`.
3. Mount `/usr/lib/wsl` and put `/usr/lib/wsl/lib` on `LD_LIBRARY_PATH`. The
   image already contains `d3d12_dri.so`, but it cannot load without
   `libd3d12core.so` and `libdxcore.so`, which are **not** in the container.
4. `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`. On a dual-GPU laptop Mesa otherwise
   selects the Intel iGPU and `gz sim` aborts inside Intel's own WSL driver with
   an LLVM fatal error in `libigc.so`. That crash looks like a Gazebo bug and is
   not one.

All four are in the `igvc_gazebo` service. Verify with
`scripts/gazebo/render_check.sh`, never with `nvidia-smi`, which reports CUDA
working while OpenGL is on a software rasteriser.

This applies to any container doing GPU rendering, not just this one.
`igvc_humble_fused_drive` and the ZED images would render `rviz2` on `llvmpipe`
today; acceptable for RViz, but do not treat it as a GPU-accurate preview.

### 10.3 Trap two: Docker Desktop has no display, WSL2 does

Docker Desktop's own Linux VM contains **no display**: `/tmp/.X11-unix` is empty
and there is no WSLg in it. A container launched from PowerShell therefore can
never show a window, regardless of GPU configuration.

Launch the same container from a **WSL2 shell** and the distro's WSLg sockets
mount through. The Gazebo GUI then opens as an ordinary Windows window on the
same D3D12 NVIDIA path. Verified through the compose service:

```text
DISPLAY=:0
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
```

This settles the dual-boot-versus-WSL2 question in favour of **WSL2**: no second
operating system, no X server, no change to a working Docker setup. It is also
the workflow §4.2 of the setup guide already recommended for `rviz2`.

`DISPLAY` is passed as `${DISPLAY:-}`, so the service still runs headless from
PowerShell with no error. Verified both ways after the change.

**Windows 10:** WSLg ships with the Microsoft Store version of WSL, which
supports Windows 10 build 19044 (22H2) and newer as well as Windows 11. It is
not in the older in-box WSL. `wsl --update` moves to the Store version;
`wsl --version` confirms it. The setup guide's stated minimum is already 22H2,
so the Windows 10 machine qualifies, but this has not been tested on it.

### 10.4 Verification

Through the committed compose service, not hand-built `docker run` commands:

- `render_check.sh` from PowerShell: `RESULT: HARDWARE RENDERING (NVIDIA)`
- `render_check.sh` from WSL2: same, plus `DISPLAY=:0` and a visible GUI
- `bridge_smoke_test.sh`: camera to `sensor_msgs/Image`, lidar to
  `sensor_msgs/LaserScan`, `/clock` to `rosgraph_msgs/Clock`, 3/3 pass
- Sensor budget: 3x RGBD at 1280x720/30 Hz gives about 7 Hz each and is below
  spec; 3x at 640x360/15 Hz gives about 11 Hz each with physics at real time

Known and harmless: `gz sim` segfaults on **teardown** (exit 139) on this render
path. Runs themselves are clean.

### 10.5 Documentation updated alongside

- `docs/GAZEBO_SETUP.md`, new
- `docs/IGVC_2027_Docker_Setup_Guide.docx`: new section 4.8, four new
  troubleshooting entries, Part 6 rewritten for Gazebo plus RViz and Windows 10,
  a WSL Store step in 2.2, a requirements bullet in 2.1, and a Gazebo block in
  the quick reference
- `README.md`: `igvc_gazebo` in the service table, GPU notes, and the WSL2
  Gazebo commands

---

## 11. Consolidating on ROS 2 Jazzy

**Date:** 2026-09-15. This reverses the distro choice behind §2, §4 and §10.

### 11.1 Why, and why it took this long to notice

Verified against the REP 2000 source, not from memory:

```text
Humble Hawksbill (May 2022 - May 2027)
Jazzy Jalisco   (May 2024 - May 2029)
```

The competition is **4 to 8 June 2027**. Humble reaches end of life the month
before it.

This is precisely the argument §10 used to reject Gazebo Fortress, which also
ends May 2027. That analysis applied the EOL test to the simulator and never
turned it on the distro underneath, then went on to build Harmonic — the
correct simulator — on top of a distro with the identical problem. Jazzy also
covers the 2028 team, which matters given annual turnover.

The 2027 team independently reached the same place. Per the coding-discussion
log of 2026-09-14 and 15, they standardised DevEnv on `osrf/ros:jazzy-desktop`
with a `jazzy_ws` workspace, and built `igvc_test_bringup`,
`igvc_test_description`, `igvc_simulation_interface` and `ping_location` under
it successfully.

### 11.2 What the port actually cost

Very little, because almost nothing in §10 was ROS-version-specific.

| Item | Change needed |
| --- | --- |
| Dockerfile | Rewritten, but only the apt section. ~15 lines *deleted*. |
| Compose service | 4 lines: dockerfile path, image tag, `LD_LIBRARY_PATH`, `ROS_DISTRO`. |
| `render_check.sh` | One comment. |
| `bridge_smoke_test.sh` | One line — now `/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash`. |
| `render_check.sdf`, `three_camera_load.sdf` | **None.** The `gz-sim-*-system` plugin names are identical. |
| `sensor_bench.sh` | **None.** |
| The four GPU settings | **None.** They are WSL2/Docker/GPU facts. |
| Measured camera budget | **None.** Same `gz sim` 8.15.0 build, so the numbers transfer exactly. |

### 11.3 What it bought

1. **`gz_ros2_control` stops being a source build.** §10.1 recorded it as the
   one genuine source build and the blocker on Stage 2, because
   `ros-humble-gz-ros2-control` 0.7.20 was built against `libignition-gazebo6`
   (Fortress, despite the `gz-` name). On Jazzy, `ros-jazzy-gz-ros2-control`
   1.2.20 exists and depends on the same `ros-jazzy-gz-*-vendor` packages as
   `ros_gz`. Installed, and `libgz_ros2_control-system.so` verified present in
   the image. **Available is not adopted** — RQ-06 is still open.
2. **No third-party apt repo.** The whole packages.osrfoundation.org block is
   gone: no GPG key fetch, no source list.
3. **No flavour split, so no `Conflicts:`.** `ros-jazzy-ros-gzharmonic` and
   `ros-jazzy-ros-gzfortress` do not exist — confirmed absent from the live
   noble index. There is one `ros-jazzy-ros-gz`.
4. **Harmonic becomes the supported pairing.** REP 2000 lists Fortress for
   Humble and Harmonic for Jazzy, so the same simulator goes from off-label to
   Tier 1.
5. **`more_diverging_changes` becomes a merge, not a port.** That branch is
   Jazzy code; the "adopting it is a port" warning was measured against Humble.

### 11.4 Verification

Through the compose service, on the RTX 5070 Ti Laptop, 2026-09-15:

```text
ROS_DISTRO      = jazzy
ubuntu          = 24.04.4 LTS (Noble Numbat)
gz sim versions = 8.15.0
GL_RENDERER     = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
RESULT: HARDWARE RENDERING (NVIDIA)                       exit 0

camera  -> sensor_msgs/Image       PASS
lidar   -> sensor_msgs/LaserScan   PASS
clock   -> rosgraph_msgs/Clock     PASS
passed: 3   failed: 0                                     exit 0

/opt/ros/jazzy/lib/libgz_ros2_control-system.so           present
```

GUI from a WSL2 shell confirmed visually the same day: the `render_check` world
with its full entity tree, dartsim physics, shadows, real-time factor 99.77%.
The 30-second run exited 124 (killed by `timeout`), meaning it survived rather
than crashed.

Image size 1023 MB pulled, 4.89 GB on disk, against 920 MB / 4.24 GB for the
Humble image. The ~100 MB difference is `gz_ros2_control` and the
`ros2_control` stack.

### 11.5 What moved and what deliberately did not

`docker/Dockerfile.gazebo-harmonic` moved to `docker/deprecated/` with a README
stating what replaced it. Its compose service is retained as
`igvc_gazebo_humble` under a `deprecated` profile, so it is hidden from
`docker compose up` and from bare `build`, but still reachable by explicit name
(Compose v5.5.1 auto-enables a profile when a service is named directly —
checked). Its colcon volumes are separate from the Jazzy service, the same
reasoning as the two ZED variants in §4.5.4.

`Dockerfile.humble-fused-drive` and `Dockerfile.igvc-zed-humble` have **not**
moved. They have no verified Jazzy replacement yet and they are the everyday
path; deprecating them now would leave the team with nothing. Marked "pending
port" in the README instead.

### 11.6 A correction, and what is still Humble

A first draft of the README change for this section claimed the org's
robot-side images were already Jazzy, citing the "ROS 2 Jazzy + ZED SDK 5"
comment on `igvc_jetson_zed`. That is wrong, and §5.4 of this document already
says so: the comment is **stale**, the image is Humble, and the service's own
commented-out command sources `/opt/ros/humble/setup.bash`. Compounding it,
`jetson-ros-base` is tagged `jazzy-36.4.7-2` while its Dockerfile builds Humble.
Both traps were documented before this change and both were fallen into anyway.

The accurate split:

- **Jazzy:** the 2026 competition branch, the competition robot's `jazzy_ws`,
  `dev_env`, the private `dev-zed`, and the 2027 team's DevEnv.
- **Humble:** `jetson-zed`, `jetson-ros-base`, `jetson-isaac-ros`, `isaac-ros`,
  and this repo's two remaining x86 images.

So the robot side is a genuine migration and **nothing here has started it**.
The EOL date forces it regardless of what the simulator does.
