# Running the simulator on Linux

Start at [`SETUP.md`](../../SETUP.md) if you have not picked a path yet.

Linux has two paths. **Use pixi unless you have a reason not to.**

| | pixi (native) | Docker |
| --- | --- | --- |
| What it is | ROS 2 Jazzy, Gazebo Harmonic and Nav2 from RoboStack, installed into `.pixi/` in the repo | The same image Windows uses, `igvc-gazebo-jazzy` |
| GPU | whatever your system's OpenGL driver provides: NVIDIA, Intel, AMD, or Mesa's software rasteriser | **NVIDIA only as written**: the `igvc_gazebo_linux` service sets `runtime: nvidia` |
| Verified | yes, in a clean Ubuntu 24.04 container with no ROS installed, software rendering, 2026-09-24 | **never run on a Linux machine** |
| Same as | the Mac path, command for command | the Windows path, minus WSL2 |

## pixi, the recommended path

**Prerequisites:** `git`, a working OpenGL driver (any desktop Linux has one;
`glxinfo -B` from `mesa-utils` shows which), and pixi:

```bash
curl -fsSL https://pixi.sh/install.sh | bash      # then open a new terminal
```

**Then, from the repository root:**

```bash
export ROS_DOMAIN_ID=<your number>    # see SETUP.md section 3
pixi run bootstrap
```

That installs the environment from `pixi.lock` on first use, builds the four
packages, runs `render_check.sh` and the smoke test, and prints PASS or FAIL
for each step. Everything after that is the same as on a Mac:

```bash
pixi run sim          # the simulator, with a Gazebo window
pixi shell            # a shell with ros2 on PATH, e.g. for teleop
pixi run autonomy     # the robot drives the course by itself
```

**Do not install ROS from apt as well** and source it in the same shell. Two
ROS installations on one PATH produce errors that point at neither.

**No Docker is involved,** so none of the container instructions elsewhere in
this repo apply: there is no `docker exec`, no `/root/ros2_ws`, and the
workspace lives in `.ws/` inside the repo.

## Docker, if you need the exact Windows image

For an NVIDIA machine with the NVIDIA Container Toolkit installed:

```bash
docker pull ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
docker tag ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest igvc-gazebo-jazzy:latest
docker compose -f docker-compose.yml up -d igvc_gazebo_linux
docker exec -it igvc_gazebo_linux bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh
```

`xhost +local:docker` first if you want a window. On a machine without an
NVIDIA GPU, remove `runtime: nvidia` from the service or use pixi instead.
**This path has not been run on a Linux machine by this project**; if you run
it, please record what happened in `docs/DOCKER_CHANGES.md`.
