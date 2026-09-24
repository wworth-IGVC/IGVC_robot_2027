#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Verify the native (pixi / RoboStack) path WITHOUT a Mac.
#
# For maintainers. Run it after changing pixi.toml, pixi.lock, igvc_env.sh or
# any script the native path uses, and before merging. It is how the Mac path
# was verified on 2026-09-24, turned into one command.
#
# What it does, in a throwaway ubuntu:24.04 container with NO ROS installed:
#   1. installs only what a Mac already has: git, ps/pgrep/pkill, a system
#      OpenGL driver (Mesa, standing in for macOS's)
#   2. copies this working copy to an ordinary home-directory path, the way a
#      Mac owner would have cloned it, so nothing can lean on /root/ros2_ws
#   3. pixi install --locked, then the exact commands the Mac guide gives:
#      pixi run bootstrap, pixi run autonomy
#   4. deletes the container
#
# What it cannot tell you: anything specific to macOS. Metal rendering, the
# separate Gazebo GUI process, BSD userland quirks and Mac timings all need a
# real Mac and docs/setup/MACOS_CHECKLIST.md. This proves the environment
# resolves and installs, that the scripts need nothing from the container
# layout, and that the checks pass on the same package recipes.
#
# Run from the repository root, in a WSL2 or Linux shell with Docker:
#   bash scripts/gazebo/native_dryrun.sh
#
# About 10 minutes and roughly 6 GB of Docker disk while it runs. Uses
# ROS_DOMAIN_ID=91 so it cannot cross-talk with a simulator you have running.
# ---------------------------------------------------------------------------
set -o pipefail
[ -f pixi.toml ] || { echo "Run from the repository root."; exit 1; }
NAME="igvc_native_dryrun_$$"
# Git Bash on Windows: `pwd -W` gives C:/..., which Docker Desktop accepts.
# WSL2 and Linux have no -W, so this falls back to the ordinary path.
REPO_HOST="$(pwd -W 2>/dev/null || pwd)"

# Fail with a reason, not silently. From a WSL2 distro without Docker
# Desktop's integration, or with Docker Desktop stopped (it stops when the
# laptop sleeps), `docker` is a placeholder that prints advice to STDOUT and
# exits non-zero; hiding stdout below would hide the only explanation.
if ! docker info >/dev/null 2>&1; then
    echo "FAIL: docker is not reachable from this shell."
    echo "      Start Docker Desktop and wait for 'docker info' to answer. From WSL2,"
    echo "      also check Settings, Resources, WSL Integration for your distro."
    exit 1
fi

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1; }
trap cleanup EXIT

echo "== starting a clean ubuntu:24.04 container: $NAME =="
if ! MSYS_NO_PATHCONV=1 docker run -d --name "$NAME" --shm-size=2g \
        -e ROS_DOMAIN_ID=91 -e LIBGL_ALWAYS_SOFTWARE=1 \
        -v "$REPO_HOST:/repo_ro:ro" ubuntu:24.04 sleep infinity; then
    echo "FAIL: could not start the container (output above)."
    exit 1
fi

MSYS_NO_PATHCONV=1 docker exec "$NAME" bash -c '
set -o pipefail
t() { date +%s; }
say() { printf "\n== %s ==\n" "$*"; }

say "system packages a Mac already has"
apt-get update -qq >/dev/null && apt-get install -y -qq curl ca-certificates git procps \
    libegl-mesa0 libgl1-mesa-dri libglx-mesa0 >/dev/null 2>&1 || { echo "apt failed"; exit 1; }
curl -fsSL https://pixi.sh/install.sh | PIXI_NO_PATH_UPDATE=1 bash >/dev/null 2>&1 || { echo "pixi install failed"; exit 1; }
export PATH="$HOME/.pixi/bin:$PATH"
echo "  $(pixi --version)"

say "copy the working copy to an ordinary clone path"
mkdir -p /home/tester/IGVC_robot_2027
tar -C /repo_ro --exclude=./build --exclude=./install --exclude=./log \
    --exclude=./.pixi --exclude=./.ws --exclude=./isaacsim -cf - . \
  | tar -C /home/tester/IGVC_robot_2027 -xf -
cd /home/tester/IGVC_robot_2027 || exit 1
git config --global --add safe.directory "*"

say "pixi install --locked"
T0=$(t); pixi install --locked > /tmp/install.log 2>&1 || { tail -30 /tmp/install.log; echo "FAIL: pixi install"; exit 1; }
echo "  ok in $(( $(t) - T0 ))s"

say "pixi run bootstrap"
T0=$(t); pixi run bootstrap > /tmp/boot.log 2>&1; B=$?
grep -E "PASS|WARN|FAIL|TIER|topic checks|DRIVE|FRAME" /tmp/boot.log | sed "s/^/  /"
echo "  bootstrap exit=$B in $(( $(t) - T0 ))s"

say "pixi run autonomy"
T0=$(t); pixi run autonomy > /tmp/auto.log 2>&1; A=$?
grep -E "distance travelled|sim time covered|clearance|MOVED|PROGRESS|IN LANE|ON THE SLAB|AUTONOMY CHECK" /tmp/auto.log | sed "s/^/  /"
echo "  autonomy exit=$A in $(( $(t) - T0 ))s"

say "RESULT"
if [ "$B" -eq 0 ] && [ "$A" -eq 0 ]; then echo "  NATIVE DRY RUN: PASS"; exit 0; fi
echo "  NATIVE DRY RUN: FAIL (bootstrap=$B autonomy=$A)"; exit 1
'
RC=$?
echo "(container $NAME removed)"
exit $RC
