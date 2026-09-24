# Maintaining the simulator environments

**For whoever owns setup this year.** Members do not need this page; they need
[`SETUP.md`](../SETUP.md).

The simulator reaches a member's machine in one of two ways, and both must
carry the same software:

| Route | Who uses it | Defined by | Delivered as |
| --- | --- | --- | --- |
| **Docker image** | Windows (all), Linux on Docker, the Mac fallback | `docker/Dockerfile.gazebo-jazzy` | `ghcr.io/wworth-igvc/igvc-gazebo-jazzy`, public on GHCR |
| **pixi environment** | Mac (Apple Silicon), native Linux | `pixi.toml` + `pixi.lock` | built on each machine by `pixi install` from RoboStack and conda-forge |

**The image contains no project code.** The repository is bind-mounted at
runtime, so a code change never needs a new image. Rebuild only when the
*installed packages* change. The same holds for pixi: code changes need
nothing; dependency changes need a new lock.

## The rule that keeps the two routes honest

**A ROS dependency added to one route is added to the other in the same
commit**, or the commit says why not. Today's deliberate differences:

| Package | Docker | pixi | Why |
| --- | --- | --- | --- |
| `ros-jazzy-twist-stamper` | yes | **no** | RoboStack does not package it; only the real-robot and Isaac launch files use it, never the Gazebo path |
| `ros-jazzy-gz-ros2-control`, `ros-jazzy-simulation-interfaces` | yes | no | unused by the Gazebo path; RQ-06 is undecided. Add to both if RQ-06 picks `gz_ros2_control` |
| `coreutils`, `compilers`, `cmake` | from the base image | explicit | macOS ships no GNU `timeout` and no toolchain inside the environment |
| gz-sim version | **8.15.0** (ROS vendor packages) | **8.10.0** (conda-forge) | Same Harmonic series; each channel ships what it ships. Recorded so a behaviour difference between routes has an obvious first suspect |

## Changing the pixi environment

```bash
# edit pixi.toml, then:
pixi lock                                   # re-resolve for osx-arm64 AND linux-64
bash scripts/gazebo/native_dryrun.sh        # clean-container proof, ~10 min
git add pixi.toml pixi.lock                 # always together
```

`pixi lock` resolves **both** platforms without installing either, so a
package missing on macOS fails here, on Windows or Linux, before any Mac owner
sees it. That is how `ros-jazzy-twist-stamper` was caught. `native_dryrun.sh`
then installs the Linux side in a container with no ROS and runs the Mac
guide's exact commands. **It cannot test anything macOS-specific**, so after a
dependency change ask a Mac owner to rerun steps 1 to 8 of
[`docs/setup/MACOS_CHECKLIST.md`](setup/MACOS_CHECKLIST.md).

`pixi update` moves every package to the newest version the constraints allow.
Do it deliberately, not as a side effect, and run all the gates afterwards.

## Changing the Docker image

1. Edit `docker/Dockerfile.gazebo-jazzy`. Check free disk first; stop below
   40 GB, because `docker_data.vhdx` grows and never shrinks on Windows 11
   Home.
2. Build and reset the volumes, which otherwise keep serving the old workspace:
   ```bash
   docker compose -f docker-compose.windows.yml build igvc_gazebo
   docker compose -f docker-compose.windows.yml down -v
   docker compose -f docker-compose.windows.yml up -d igvc_gazebo
   ```
3. **All three gates must pass** in the new container: `render_check.sh`,
   `bringup_smoke_test.sh`, `autonomy_check.sh`.
4. **Audit before publishing**, because a public package cannot be unpublished
   in practice. The checklist is `docs/DOCKER_CHANGES.md` section 16.4: no
   secrets, no private layers, no project code baked in.
5. Tag with the date and `latest`, and publish. Only the namespace owner can
   push (`wworth-IGVC`), with a **classic** personal access token carrying
   `write:packages`; fine-grained tokens have no Packages permission:
   ```bash
   docker tag igvc-gazebo-jazzy:latest ghcr.io/wworth-igvc/igvc-gazebo-jazzy:YYYY-MM-DD
   docker tag igvc-gazebo-jazzy:latest ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
   docker login ghcr.io -u wworth-IGVC        # paste the token at the prompt
   docker push ghcr.io/wworth-igvc/igvc-gazebo-jazzy:YYYY-MM-DD
   docker push ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
   ```
6. **Verify as an outsider.** This, not whether a web page loads, is the test
   of public visibility:
   ```bash
   docker logout ghcr.io
   docker manifest inspect ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
   ```
7. Record both digests (the index and the amd64 manifest) in
   `docs/DOCKER_CHANGES.md` section 16.

**The image is amd64 only.** A native arm64 build would make the Mac *Docker
fallback* run without emulation. It is not needed for the Mac *route*, which is
pixi, and it has not been built. If it is ever wanted, build arm64 on native
arm64 hardware rather than under emulation, and publish it into the same tag
alongside the existing amd64 manifest so Windows users keep byte-identical
bits.

## Before anything merges into `main`

- **Windows gates:** all three pass on the merge result, in `igvc_gazebo`.
- **Native dry run:** `native_dryrun.sh` passes, if the change touches
  `pixi.toml`, `pixi.lock`, `scripts/gazebo/` or a launch file.
- The platform guides in `docs/setup/` still match what the scripts print.

## Planned: perception on Jazzy, on both routes

The one piece of the simulator stack still on ROS 2 Humble is YOLOPv2 lane
segmentation, which runs in the Humble `igvc-humble-fused-drive` image and
talks to the Jazzy simulator across a distro boundary with an open
deserialisation defect (P0-1 in `docs/GAZEBO_TODO.md`). The plan is to remove
Humble from the simulator path entirely. When that happens, keep it additive on
both routes: a pixi **feature** (for example `[feature.perception]` with
PyTorch, which on Apple Silicon can use the GPU through MPS) and a Docker image
built `FROM` the Gazebo image, rather than a second, unrelated environment.
