#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One command to get from a fresh clone to a verified simulator.
#
# RUN THIS FROM A WSL2 UBUNTU SHELL, in the repository root, AFTER the
# prerequisite check in scripts/setup-windows.ps1 has passed. It is the only
# bootstrap entry point; start_sim.sh is the thing you run afterwards, from
# inside the container, to actually see the robot.
#
#   bash scripts/gazebo/bootstrap.sh
#
# What it does, in order, printing PASS or FAIL for each step and stopping at
# the first failure:
#
#   1. sanity: are we in the right repo, is docker reachable, is there disk
#   2. git submodule update --init --recursive
#   3. the image: load it from a tar if one is given, otherwise build it
#   4. start the container
#   5. colcon build the four packages the bringup needs
#   6. render_check.sh      is it on the GPU, or silently on the CPU
#   7. bringup_smoke_test.sh  does the topic contract hold, does it drive
#
# Options, as environment variables:
#
#   IMAGE_TAR=/mnt/d/igvc-gazebo-jazzy.tar
#                 load the image from this file instead of building it. This
#                 is the offline path: no pull, no build, no campus wifi.
#   SKIP_SMOKE=1  stop after render_check.sh. Saves about 4 minutes.
#   FORCE_BUILD=1 rebuild the image even if it is already present.
#
# It is deliberately noisy on failure and silent-free on success: every step
# prints a line, and a step that cannot run says so rather than being skipped.
# ---------------------------------------------------------------------------
set -o pipefail   # NOT set -u: it breaks /opt/ros/*/setup.bash, which reads
                  # AMENT_TRACE_SETUP_FILES unguarded.

COMPOSE_FILE=docker-compose.windows.yml
SERVICE=igvc_gazebo
CONTAINER=igvc_gazebo
IMAGE=igvc-gazebo-jazzy:latest
REPO_IN_CONTAINER=/root/ros2_ws/src/IGVC_robot_2026

# The Gazebo-only path needs this much free disk. Derived in
# docs/GAZEBO_QUICKSTART.md section 1: 5.6 GB for the unpacked image, 1.2 GB
# for the tar if you are loading one, about 0.6 GB for the clone and its
# submodules, and the rest is headroom for the WSL2 VM and a second image
# version. Building from source instead of loading a tar leaves build cache
# behind, which is why that number is higher.
NEED_GB_LOAD=12
NEED_GB_BUILD=25

STEP=0
FAILED=""

say()  { printf '\n%s\n' "$*"; }
pass() { printf '  PASS  %s\n' "$*"; }
info() { printf '        %s\n' "$*"; }
fail() {
    printf '\n  FAIL  %s\n' "$*"
    printf '\n  Stopped at step %s. Nothing after this point has run.\n' "$STEP"
    if [ -n "$FAILED" ]; then printf '  %s\n' "$FAILED"; fi
    printf '\n  docs/GAZEBO_QUICKSTART.md section 9 lists the usual causes.\n\n'
    exit 1
}
step() { STEP=$((STEP + 1)); printf '\n==> step %s: %s\n' "$STEP" "$*"; }

printf '\n'
printf '  IGVC 2027 Gazebo bootstrap\n'
printf '  ---------------------------------------------------------------\n'

# ---------------------------------------------------------------------------
step "checking where we are and what we have"
# ---------------------------------------------------------------------------
if [ ! -f "$COMPOSE_FILE" ]; then
    FAILED="Run it from the repository root, the directory that contains $COMPOSE_FILE."
    fail "$COMPOSE_FILE is not here. cwd is $(pwd)"
fi
pass "in the repository root"

case "$(pwd)" in
    /mnt/*) pass "running from a WSL2 shell, so GUI windows will work later" ;;
    *) info "NOTE: cwd is not under /mnt, so this may not be a WSL2 shell."
       info "Headless checks still work. A Gazebo window will not."
       info "See docs/GAZEBO_QUICKSTART.md section 0." ;;
esac

if ! command -v docker >/dev/null 2>&1; then
    FAILED="Docker Desktop: Settings, Resources, WSL Integration, enable your distro, Apply and Restart."
    fail "the 'docker' command is not on PATH in this shell"
fi
if ! docker info >/dev/null 2>&1; then
    FAILED="Start Docker Desktop and wait for 'docker info' to answer, then run this again."
    fail "the docker daemon is not responding"
fi
pass "docker is reachable  ($(docker --version 2>/dev/null))"

if [ -n "$IMAGE_TAR" ]; then NEED_GB=$NEED_GB_LOAD; else NEED_GB=$NEED_GB_BUILD; fi
FREE_GB=$(df -BG --output=avail . 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -z "$FREE_GB" ]; then
    info "could not read free disk; wanted at least ${NEED_GB} GB. Continuing."
elif [ "$FREE_GB" -lt "$NEED_GB" ]; then
    FAILED="Reclaim space, or free it in Docker Desktop with: docker builder prune"
    fail "not enough free disk. Wanted ${NEED_GB} GB, found ${FREE_GB} GB"
else
    pass "free disk: wanted ${NEED_GB} GB, found ${FREE_GB} GB"
fi

# ---------------------------------------------------------------------------
step "submodules"
# ---------------------------------------------------------------------------
# This is not optional and it is the single most common way a fresh clone
# fails. zed_description is a submodule and colcon builds it by name, and
# IGVC_track_generator holds the track data every navigation node reads. A
# plain 'git clone' without --recurse-submodules leaves both empty.
if ! git submodule update --init --recursive 2>&1 | tail -20; then
    FAILED="Check network access to github.com, then run: git submodule update --init --recursive"
    fail "git submodule update failed"
fi
EMPTY=$(git submodule status | grep -c '^-')
if [ "$EMPTY" -ne 0 ]; then
    FAILED="Run: git submodule update --init --recursive"
    fail "$EMPTY submodule(s) are still empty. The build will fail without them"
fi
TOTAL=$(git submodule status | wc -l)
pass "all $TOTAL submodules populated"
for req in IGVC_track_generator/track_points.json src/zed-description/package.xml; do
    if [ ! -f "$req" ]; then
        FAILED="A submodule is present but empty. Try: git submodule update --init --recursive --force"
        fail "$req is missing, and the simulator needs it"
    fi
done
pass "track data and zed_description are on disk"

# ---------------------------------------------------------------------------
step "the image"
# ---------------------------------------------------------------------------
HAVE_IMAGE=$(docker image ls -q "$IMAGE" 2>/dev/null)

if [ -n "$IMAGE_TAR" ]; then
    if [ ! -f "$IMAGE_TAR" ]; then
        FAILED="Check the path. From WSL2 a USB drive at D: is /mnt/d, so: IMAGE_TAR=/mnt/d/igvc-gazebo-jazzy.tar"
        fail "IMAGE_TAR=$IMAGE_TAR does not exist"
    fi
    info "loading $IMAGE_TAR  ($(du -h "$IMAGE_TAR" | cut -f1)). No pull, no build."
    if ! docker load -i "$IMAGE_TAR"; then
        FAILED="If the tar is truncated, copy it from the USB drive again."
        fail "docker load failed"
    fi
    pass "image loaded from the tar"
elif [ -n "$HAVE_IMAGE" ] && [ "$FORCE_BUILD" != "1" ]; then
    pass "$IMAGE is already present, skipping the build  (FORCE_BUILD=1 to rebuild)"
else
    info "building $IMAGE. Ten to twenty minutes the first time."
    if ! docker compose -f "$COMPOSE_FILE" build "$SERVICE"; then
        FAILED="Re-run it first: partial layers are cached and a network hiccup mid-download is the usual cause."
        fail "the image build failed"
    fi
    pass "image built"
fi

if [ -z "$(docker image ls -q "$IMAGE" 2>/dev/null)" ]; then
    FAILED="Neither the load nor the build produced it. Check 'docker image ls'."
    fail "$IMAGE is still not present after the image step"
fi

# ---------------------------------------------------------------------------
step "starting the container"
# ---------------------------------------------------------------------------
# 'compose up -d' honours container_name, so there is exactly one container
# called igvc_gazebo and 'docker exec' can find it. 'compose run' makes a new
# container with a random name every time, which is how six simulators once
# ran at once.
if ! docker compose -f "$COMPOSE_FILE" up -d "$SERVICE" 2>&1 | tail -5; then
    FAILED="Check 'docker logs $CONTAINER'. Usually Docker Desktop is out of memory."
    fail "the container would not start"
fi
if [ -z "$(docker ps -q -f "name=^${CONTAINER}$")" ]; then
    FAILED="Check 'docker logs $CONTAINER'."
    fail "$CONTAINER is not running after 'compose up -d'"
fi
pass "$CONTAINER is up"

# ---------------------------------------------------------------------------
step "building the workspace"
# ---------------------------------------------------------------------------
# Four packages, not three: zed_description is in the list because
# igvc_test_description depends on it, and leaving it out fails outright.
info "colcon build: zed_description igvc_test_description igvc_test_bringup igvc_lane_detection"
if ! docker exec "$CONTAINER" bash -c '
    set -o pipefail
    source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
    cd /root/ros2_ws || exit 1
    colcon build --symlink-install \
        --base-paths '"$REPO_IN_CONTAINER"'/src \
        --packages-select zed_description igvc_test_description \
                          igvc_test_bringup igvc_lane_detection \
        > /tmp/bootstrap_colcon.log 2>&1
    RC=$?
    if [ $RC -ne 0 ]; then
        echo "colcon exited $RC"
        tail -40 /tmp/bootstrap_colcon.log
        exit $RC
    fi
'; then
    FAILED="The colcon output above is the real error. A missing submodule shows up here first."
    fail "the workspace build failed"
fi
pass "four packages built"

# ---------------------------------------------------------------------------
step "render_check.sh: is it on the GPU, or silently on the CPU"
# ---------------------------------------------------------------------------
# This goes before the smoke test on purpose. Gazebo falls back to software
# rendering without erroring, and every camera number from such a run is
# meaningless. The script sources ROS itself, so a bare docker exec is enough.
RENDER_OUT=$(docker exec "$CONTAINER" bash -c \
    "cd /root/ros2_ws && bash $REPO_IN_CONTAINER/scripts/gazebo/render_check.sh" 2>&1)
RENDER_RC=$?
echo "$RENDER_OUT" | grep -E "GL_RENDERER|RESULT:|camera topic|gpu_lidar" | sed 's/^/        /'
if [ $RENDER_RC -ne 0 ]; then
    FAILED="Full output: docker exec $CONTAINER bash -c 'cd /root/ros2_ws && bash $REPO_IN_CONTAINER/scripts/gazebo/render_check.sh'"
    fail "render_check.sh exited $RENDER_RC"
fi
if echo "$RENDER_OUT" | grep -q "SOFTWARE FALLBACK"; then
    printf '\n  WARN  Gazebo is rendering on the CPU (llvmpipe), not the GPU.\n'
    info "The simulator still runs and navigation, lidar and control work are"
    info "all still valid. Cameras are not: do not take camera or timing"
    info "measurements, and run with CAMERAS=none."
    info "Causes, in order: WSL Integration off for your distro; no NVIDIA"
    info "driver on WINDOWS; MESA_D3D12_DEFAULT_ADAPTER_NAME pinned to an"
    info "adapter you do not have. See GAZEBO_QUICKSTART.md section 1A."
    info "This is a warning, not a failure. Continuing."
else
    pass "hardware rendering, and both sensors are publishing"
fi

# ---------------------------------------------------------------------------
step "bringup_smoke_test.sh: the topic contract, and does it drive"
# ---------------------------------------------------------------------------
if [ "$SKIP_SMOKE" = "1" ]; then
    info "SKIP_SMOKE=1, so this was not run. The simulator is NOT verified."
else
    info "about four minutes. It commands a velocity and compares the"
    info "odometry against Gazebo's own ground truth."
    SMOKE_OUT=$(docker exec "$CONTAINER" bash -c \
        "cd /root/ros2_ws && bash $REPO_IN_CONTAINER/scripts/gazebo/bringup_smoke_test.sh" 2>&1)
    SMOKE_RC=$?
    echo "$SMOKE_OUT" | grep -E "topic checks passed|DRIVE TEST|FRAME TEST|heading disagreement|distance ratio" \
        | sed 's/^/        /'
    if [ $SMOKE_RC -ne 0 ]; then
        printf '\n'
        echo "$SMOKE_OUT" | grep -E "FAIL" | head -20 | sed 's/^/        /'
        FAILED="Re-run it by hand for the full output, which names the failing topic."
        fail "bringup_smoke_test.sh exited $SMOKE_RC"
    fi
    pass "the topic contract holds and the robot drives on /cmd_vel"
fi

# ---------------------------------------------------------------------------
printf '\n'
printf '  ---------------------------------------------------------------\n'
printf '  Bootstrap complete. The simulator is verified on this machine.\n'
printf '\n'
printf '  See the robot drive the course by itself:\n'
printf '\n'
printf '    docker exec -it %s bash -c "NAV=1 bash %s/scripts/gazebo/start_sim.sh"\n' \
    "$CONTAINER" "src/IGVC_robot_2026"
printf '\n'
printf '  Drive it yourself, from a SECOND WSL2 shell:\n'
printf '\n'
printf '    docker exec -it %s bash\n' "$CONTAINER"
printf '    source /root/ros2_ws/install/setup.bash\n'
printf '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
printf '\n'
