# Docker environment — change record

Documents every change made to the Docker setup in this fork, the reason for
each one, and how it was verified. Written so a reviewer can check the claims
rather than take them on trust.

- **Fork:** `wworth-IGVC/IGVC_robot_2027` (of `Gold-Rush-Robotics/IGVC_robot_2026`)
- **Baseline commit:** `a4b7433`
- **First change commit:** `e1b118d`
- **Tested on:** Windows 11 (build 26200), Docker Engine 29.7.2, Compose v5.5.1,
  WSL2 / Ubuntu 26.04, NVIDIA RTX 5070 Ti Laptop (Blackwell, sm_120)

---

## 1. Summary

The `igvc_humble_fused_drive` Docker image could not build **any** ROS package.
A `colcon build` of the workspace produced:

```text
Summary: 0 packages finished
  1 package failed: debug_gui
  21 packages not processed
```

After the fixes below:

```text
Summary: 22 packages finished [4min 15s]
  2 packages had stderr output: odrive_ros2_control sllidar_ros2
COLCON_EXIT=0
```

The two remaining stderr packages emit compiler warnings only — see §5.3.

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
docker compose -f docker-compose.windows.yml build
```

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

### 4.2.1 Why SDK 5.3.0 and not 5.1.0

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

Pinned to **5.3.0** instead, which satisfies the requirement and matches
`docker_images/jetson-zed` (`ZED_SDK_MAJOR=5 MINOR=3 PATCH=0`) so x86 and Jetson
run the same SDK.

**Confirmed fixed.** The rebuild with 5.3.0 compiled `zed_components` in 1 min
29 s and finished the workspace with `Summary: 6 packages finished [2min 2s]`,
exit 0. The installed SDK was verified in the image rather than assumed:

```text
$ grep PACKAGE_VERSION /usr/local/zed/zed-config-version.cmake
set(PACKAGE_VERSION "5.3.0")

$ ros2 pkg list | grep -i zed
zed_components  zed_debug  zed_description
zed_msgs        zed_ros2   zed_wrapper
```

> Worth noting: `jetson-zed` clones **upstream** `stereolabs/zed-ros2-wrapper`,
> not the org's `zed-ros2-wrapper-2025` fork. The Dockerfile exposes
> `ZED_WRAPPER_REPO` / `ZED_WRAPPER_BRANCH` build args so either can be used
> without editing the recipe.

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

| Task | Image | Size |
| --- | --- | --- |
| Everyday development | `igvc_humble_fused_drive` | 3.98 GB |
| ZED camera code, robot machines | `igvc_zed_humble` | 9.7 GB |

Both sizes are measured from `docker image ls`, not estimated. The ZED image
came in well under the 15–20 GB originally projected, because `skip_cuda` and
`skip_tools` keep the SDK installer from duplicating CUDA or adding the GUI
tools.

On Windows the smaller image is the right default: Docker Desktop cannot pass
through USB, so a ZED camera is unusable there, and the 22 workspace packages
build without the SDK (proven — the 4 GB image has no SDK and builds 22/22).

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

### 6.3 Documentation

- `README.md` — Windows section covering WSL2 (Option A) vs PowerShell
  (Option B), the setup script, GPU/Blackwell notes, ZED limitations, and the
  volume-naming difference in a fork named `IGVC_robot_2027`.
- `docs/IGVC_2027_Docker_Setup_Guide.docx` — step-by-step guide for teammates
  who have never used Docker.
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
| Image builds clean | `docker build -f docker/Dockerfile.igvc-zed-humble .` | exit 0, **18.5 min**, **9.7 GB** |
| ZED SDK installed | `grep PACKAGE_VERSION /usr/local/zed/zed-config-version.cmake` | `5.3.0` |
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

---

## 8. Outstanding

1. **`dev-zed` access.** Still private. Either have the package made public /
   granted to team accounts, or adopt `igvc-zed-humble` (§4) and retire it.
2. **Publish `igvc-zed-humble`.** Building it per-developer costs 9.7 GB and
   ~18.5 minutes each (measured; see §7). Push it to GHCR once and have
   everyone pull instead.
3. **Correct the `jetson-ros-base` tag** upstream — `jazzy-36.4.7-2` contains
   Humble (§5.4).
4. **Fix the stale Jazzy comments** in `docker-compose.jetson.yml` (§5.4).
5. **Confirm submodule remotes** — upstream Stereolabs vs the org forks (§5.4).
6. **ZED camera on Windows.** `usbipd-win` can attach USB devices into WSL2, but
   ZED cameras are high-bandwidth USB3 and USB/IP handles that poorly. Test
   before relying on it.
