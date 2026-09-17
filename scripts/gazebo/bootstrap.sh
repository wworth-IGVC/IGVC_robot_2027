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
#   3. the image: pull it from GHCR (no login needed, the package is public)
#   4. start the container
#   5. colcon build the four packages the bringup needs
#   6. render_check.sh      which GPU tier is this machine, if any
#   7. bringup_smoke_test.sh  does the topic contract hold, does it drive
#
# Options, as environment variables:
#
#   BUILD_IMAGE=1   build the image from the Dockerfile instead of pulling it.
#                   The documented fallback for when GHCR is unreachable.
#                   Ten to twenty minutes instead of a few.
#   IMAGE_REF=...   pull a different tag, e.g. a dated one instead of latest.
#   SKIP_SMOKE=1    stop after render_check.sh. Saves about 4 minutes.
#   ALLOW_TIER_C=1  continue past a software-rendering machine rather than
#                   stopping to explain it. The tier C path still works; this
#                   only silences the pause.
#
# It is deliberately noisy on failure: every step prints a line, and a step
# that cannot run says so rather than being skipped. It will never tell you
# the GPU is working when it is not.
# ---------------------------------------------------------------------------
set -o pipefail   # NOT set -u: it breaks /opt/ros/*/setup.bash, which reads
                  # AMENT_TRACE_SETUP_FILES unguarded.

# These three default to the Windows values, so the Windows path is byte for
# byte what it was. They are overridable only so the macOS path can reuse this
# script instead of duplicating it; see docs/MAC_SETUP.md. Nothing that does
# not set them can behave differently.
COMPOSE_FILE="${COMPOSE_FILE:-docker-compose.windows.yml}"
SERVICE="${SERVICE:-igvc_gazebo}"
CONTAINER="${CONTAINER:-igvc_gazebo}"
# Empty on Windows, so `docker pull` is invoked exactly as before. macOS sets
# this to linux/amd64 because the published image has no arm64 manifest and an
# explicit platform is more predictable than a mismatch warning.
PULL_PLATFORM="${PULL_PLATFORM:-}"
LOCAL_IMAGE=igvc-gazebo-jazzy:latest
REGISTRY_IMAGE="${IMAGE_REF:-ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest}"
REPO_IN_CONTAINER=/root/ros2_ws/src/IGVC_robot_2026

# Free disk the Gazebo path needs, measured on this project. See
# docs/GAZEBO_QUICKSTART.md section 1 for the derivation:
#   5.57 GB  the image, unpacked on disk
#   0.60 GB  the clone and its nine submodules, including .git
#   ~4 GB    Docker Desktop itself plus the WSL2 Ubuntu distro, on a blank
#            Windows install
# plus headroom, because docker_data.vhdx grows and never shrinks and
# Windows 11 Home has no Hyper-V to compact it.
NEED_GB_PULL=20
NEED_GB_BUILD=30

STEP=0
FAILED=""

say()  { printf '\n%s\n' "$*"; }
pass() { printf '  PASS  %s\n' "$*"; }
info() { printf '        %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*"; }
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
    *) warn "cwd is not under /mnt, so this may not be a WSL2 shell."
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

if [ "$BUILD_IMAGE" = "1" ]; then NEED_GB=$NEED_GB_BUILD; else NEED_GB=$NEED_GB_PULL; fi
FREE_GB=$(df -BG --output=avail . 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -z "$FREE_GB" ]; then
    FREE_GB=$(df -k . 2>/dev/null | tail -1 | awk '{print int($4/1048576)}')
fi
if [ -z "$FREE_GB" ]; then
    warn "could not read free disk. WANTS ${NEED_GB} GB. Continuing anyway."
elif [ "$FREE_GB" -lt "$NEED_GB" ]; then
    FAILED="Reclaim space with 'docker builder prune' then 'docker image prune -a'."
    fail "not enough free disk.  WANTS ${NEED_GB} GB   FOUND ${FREE_GB} GB"
else
    pass "free disk:  WANTS ${NEED_GB} GB   FOUND ${FREE_GB} GB"
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
HAVE_IMAGE=$(docker image ls -q "$LOCAL_IMAGE" 2>/dev/null)

if [ "$BUILD_IMAGE" = "1" ]; then
    info "BUILD_IMAGE=1, building from the Dockerfile. Ten to twenty minutes."
    if ! docker compose -f "$COMPOSE_FILE" build "$SERVICE"; then
        FAILED="Re-run it first: partial layers are cached and a network hiccup mid-download is the usual cause."
        fail "the image build failed"
    fi
    pass "image built"
elif [ -n "$HAVE_IMAGE" ]; then
    pass "$LOCAL_IMAGE is already present, skipping the pull"
    info "to force a fresh pull: docker rmi $LOCAL_IMAGE   (see the VHDX warning in the quickstart first)"
else
    info "pulling $REGISTRY_IMAGE"
    info "about 1.2 GB over the wire, unpacking to about 5.6 GB on disk."
    info "The package is public, so no docker login is needed."
    if [ -n "$PULL_PLATFORM" ]; then
        info "pulling with --platform $PULL_PLATFORM"
        PULL_CMD="docker pull --platform $PULL_PLATFORM $REGISTRY_IMAGE"
    else
        PULL_CMD="docker pull $REGISTRY_IMAGE"
    fi
    if ! $PULL_CMD; then
        FAILED="If GHCR is unreachable, build instead:  BUILD_IMAGE=1 bash scripts/gazebo/bootstrap.sh"
        fail "docker pull failed for $REGISTRY_IMAGE"
    fi
    # The compose file refers to the short local name, so give it that name.
    docker tag "$REGISTRY_IMAGE" "$LOCAL_IMAGE" || {
        FAILED="Unexpected: the pull succeeded but tagging did not."
        fail "could not tag $REGISTRY_IMAGE as $LOCAL_IMAGE"
    }
    pass "image pulled and tagged as $LOCAL_IMAGE"
fi

if [ -z "$(docker image ls -q "$LOCAL_IMAGE" 2>/dev/null)" ]; then
    FAILED="Neither the pull nor the build produced it. Check 'docker image ls'."
    fail "$LOCAL_IMAGE is still not present after the image step"
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
step "render_check.sh: which GPU tier is this machine"
# ---------------------------------------------------------------------------
# This goes before the smoke test on purpose. Gazebo falls back to software
# rendering without erroring, and every camera number from such a run is
# meaningless. render_check.sh writes the tier to /tmp/render_tier so this
# script reads its verdict rather than re-deriving it and possibly disagreeing.
RENDER_OUT=$(docker exec "$CONTAINER" bash -c \
    "cd /root/ros2_ws && bash $REPO_IN_CONTAINER/scripts/gazebo/render_check.sh" 2>&1)
RENDER_RC=$?
echo "$RENDER_OUT" | grep -E "ADAPTER:|RESULT:|YOUR TIER:|camera topic|gpu_lidar" | sed 's/^/        /'

# Read the tier from render_check.sh's own output first. The tier file is a
# secondary source, because reading it needs a container path and Git Bash
# rewrites /tmp/render_tier into a Windows path (MSYS path conversion). That
# made this report TIER UNKNOWN on the line directly after render_check.sh
# had printed YOUR TIER: A. It only bit the undocumented Git Bash
# invocation, not the documented WSL2 one, which is why it survived the
# clean-clone run. Parsing the captured output works in either shell.
TIER=$(printf '%s' "$RENDER_OUT" | sed -n 's/.*YOUR TIER:[[:space:]]*\([A-Za-z]*\).*/\1/p' | head -1)
if [ -z "$TIER" ]; then
    TIER=$(MSYS_NO_PATHCONV=1 docker exec "$CONTAINER" cat /tmp/render_tier 2>/dev/null | tr -d '[:space:]')
fi
[ -z "$TIER" ] && TIER="unreported"

case "$TIER" in
  A)
    pass "TIER A, hardware rendering on a discrete NVIDIA adapter"
    info "This is the measured baseline. The quickstart applies as written."
    ;;
  B)
    pass "TIER B, hardware rendering on an integrated Intel or AMD adapter"
    warn "Tier B is NOT measured by this project on any machine."
    info "Your GPU is genuinely rendering, so this is a real pass. But treat"
    info "performance as unknown rather than assuming it matches tier A, and"
    info "please report your numbers back."
    ;;
  C)
    warn "TIER C, SOFTWARE RENDERING. The GPU is not reaching the container."
    info "Nothing is broken. Navigation, lidar, odometry and control work are"
    info "all still valid, because gpu_lidar falls back too and the physics"
    info "runs on the CPU either way. Cameras are the part that suffers."
    info ""
    info "This script will NOT claim your GPU works. Follow the TIER C path in"
    info "docs/GAZEBO_QUICKSTART.md: headless plus RViz, front camera only,"
    info "and do not take camera or timing measurements."
    if [ "$ALLOW_TIER_C" != "1" ]; then
        info ""
        info "Continuing to the smoke test anyway, because it is the gate that"
        info "decides whether this machine is usable. Expect it to be slow."
    fi
    ;;
  *)
    warn "TIER UNKNOWN. render_check.sh exited $RENDER_RC and reported '$TIER'."
    info "The render log named an adapter the script does not recognise, which"
    info "usually means a GPU vendor nobody here has tried. Report the ADAPTER"
    info "line above. Treat yourself as tier C until someone confirms otherwise."
    ;;
esac

# A hard guard: never let a software-rendering machine be recorded as tier A.
if [ "$TIER" = "A" ] && echo "$RENDER_OUT" | grep -q "SOFTWARE RENDERING"; then
    FAILED="This is a bug in render_check.sh, not in your machine. Report it."
    fail "internal inconsistency: tier reported A while the log says software rendering"
fi

# ---------------------------------------------------------------------------
step "bringup_smoke_test.sh: the topic contract, and does it drive"
# ---------------------------------------------------------------------------
if [ "$SKIP_SMOKE" = "1" ]; then
    warn "SKIP_SMOKE=1, so this was not run. The simulator is NOT verified."
else
    info "about four minutes on tier A, longer on tier C. It commands a"
    info "velocity and compares the odometry against Gazebo's ground truth."
    SMOKE_OUT=$(docker exec "$CONTAINER" bash -c \
        "cd /root/ros2_ws && bash $REPO_IN_CONTAINER/scripts/gazebo/bringup_smoke_test.sh" 2>&1)
    SMOKE_RC=$?
    echo "$SMOKE_OUT" | grep -E "topic checks passed|DRIVE TEST|FRAME TEST|heading disagreement|distance ratio" \
        | sed 's/^/        /'
    if [ $SMOKE_RC -ne 0 ]; then
        printf '\n'
        echo "$SMOKE_OUT" | grep -E "FAIL" | head -20 | sed 's/^/        /'
        if [ "$TIER" = "C" ]; then
            FAILED="On tier C, try the smaller camera configuration in the quickstart's tier C path before concluding anything is broken."
        else
            FAILED="Re-run it by hand for the full output, which names the failing topic."
        fi
        fail "bringup_smoke_test.sh exited $SMOKE_RC"
    fi
    pass "the topic contract holds and the robot drives on /cmd_vel"
fi

# ---------------------------------------------------------------------------
printf '\n'
printf '  ---------------------------------------------------------------\n'
if [ "$SKIP_SMOKE" = "1" ]; then
    printf '  Bootstrap finished, but SKIP_SMOKE=1 was set, so the simulator\n'
    printf '  is NOT verified. Re-run without SKIP_SMOKE before trusting it.\n'
else
    printf '  Bootstrap complete. The simulator is verified on this machine.\n'
fi
printf '  Your GPU tier: %s\n' "$TIER"
printf '\n'
if [ "$TIER" = "C" ]; then
    printf '  TIER C, so run it headless with RViz rather than with the Gazebo\n'
    printf '  window, which is unusable on a software rasteriser:\n'
    printf '\n'
    printf '    docker exec -it %s bash -c "CAMERAS=front NAV=1 HEADLESS=1 RVIZ=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"\n' "$CONTAINER"
else
    printf '  See the robot drive the course by itself:\n'
    printf '\n'
    printf '    docker exec -it %s bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"\n' "$CONTAINER"
fi
printf '\n'
printf '  Drive it yourself, from a SECOND WSL2 shell:\n'
printf '\n'
printf '    docker exec -it %s bash\n' "$CONTAINER"
printf '    source /root/ros2_ws/install/setup.bash\n'
printf '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
printf '\n'
