# scripts/gazebo/igvc_env.sh
# ---------------------------------------------------------------------------
# SOURCED, never run. The one place that knows where things live, so every
# script in this directory runs unchanged in both supported layouts:
#
#   docker  (Windows, Linux, and the macOS fallback)
#           ROS in /opt/ros/jazzy, workspace /root/ros2_ws, repo bind-mounted
#           at /root/ros2_ws/src/IGVC_robot_2026. This is the default, so the
#           Windows path behaves exactly as it did before this file existed.
#
#   pixi    (macOS, the primary Mac route; also native Linux)
#           ROS from RoboStack inside the pixi environment, which is already
#           activated by `pixi run` or `pixi shell`. pixi.toml sets every
#           IGVC_* variable below.
#
# After sourcing, these are set:
#   REPO           the repository root
#   WS             the colcon workspace (build/, install/, log/ live here)
#   IGVC_RUNTIME   docker | pixi
#   IGVC_ROS_PREFIX  where ROS itself is installed
# and these helpers exist:
#   igvc_source_ws       source the workspace overlay after a build
#   igvc_restart_hint    print how to get a clean simulator on THIS layout
#   igvc_shell_hint      print how to open a second shell on THIS layout
# ---------------------------------------------------------------------------

IGVC_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IGVC_RUNTIME="${IGVC_RUNTIME:-docker}"
REPO="${IGVC_REPO:-$(cd "$IGVC_SCRIPTS/../.." && pwd)}"
WS="${IGVC_WS:-/root/ros2_ws}"
IGVC_ROS_PREFIX="${IGVC_ROS_PREFIX:-/opt/ros/${ROS_DISTRO:-jazzy}}"

if [ "$IGVC_RUNTIME" = "pixi" ]; then
    # The pixi environment is activated by `pixi run` / `pixi shell`, which is
    # what puts ros2 on PATH. Sourcing a setup file here as well would be
    # harmless, but running outside the environment is the real mistake and
    # it should be named rather than half-worked-around.
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ERROR: ros2 is not on PATH. Run this through pixi, from the repo root:"
        echo "         pixi run smoke        (or: pixi shell, then the script)"
        return 1 2>/dev/null || exit 1
    fi
else
    # No ROS where the container keeps it means this is not the container.
    # The usual cause is a Mac or Linux user running a script directly instead
    # of through pixi; without this check `source` fails, the script carries
    # on, and the first error is a baffling "ros2: command not found" later.
    if [ ! -f "$IGVC_ROS_PREFIX/setup.bash" ]; then
        echo "ERROR: no ROS 2 at $IGVC_ROS_PREFIX, so this is not the Docker container."
        echo "       On a Mac or native Linux, run it through pixi from the repo root:"
        echo "         pixi run smoke      (or render-check, autonomy, sim)"
        echo "       On Windows, run it inside the container: see SETUP.md."
        return 1 2>/dev/null || exit 1
    fi
    # Unconditional, exactly as every script did before this file existed.
    # set -u must stay off: this setup.bash reads AMENT_TRACE_SETUP_FILES
    # unguarded.
    # shellcheck disable=SC1090
    source "$IGVC_ROS_PREFIX/setup.bash"
fi
mkdir -p "$WS" 2>/dev/null

igvc_source_ws () {
    # shellcheck disable=SC1091
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
}

# The four packages the simulator needs, and ONLY those. zed_description is
# the one people leave out; the robot description includes its meshes by
# package name, so without it the build fails on a package nobody touched.
# All four are install-only (no C++ is compiled), so this takes seconds.
IGVC_SIM_PACKAGES="zed_description igvc_test_description igvc_test_bringup igvc_lane_detection"

igvc_build () {   # optional args: a subset of packages
    local pkgs="${*:-$IGVC_SIM_PACKAGES}" rc
    # shellcheck disable=SC2086
    ( cd "$WS" && colcon build --symlink-install --base-paths "$REPO/src" \
        --packages-select $pkgs ) > /tmp/colcon.log 2>&1
    rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "FAIL: colcon build exited $rc. Last 30 lines of /tmp/colcon.log:"
        tail -30 /tmp/colcon.log
        return "$rc"
    fi
    igvc_source_ws
    return 0
}

igvc_restart_hint () {
    if [ "$IGVC_RUNTIME" = "pixi" ]; then
        echo "   close the other terminal running the simulator, or run:"
        echo "   pkill -f 'gz sim'; pkill -f 'ros2 launch'"
    else
        echo "   docker compose -f ${IGVC_COMPOSE_FILE:-docker-compose.windows.yml} restart ${IGVC_SERVICE:-igvc_gazebo}"
    fi
}

igvc_shell_hint () {
    if [ "$IGVC_RUNTIME" = "pixi" ]; then
        echo "    pixi shell            (from the repo root, in a new terminal)"
    else
        echo "    docker exec -it ${IGVC_CONTAINER:-igvc_gazebo} bash"
        echo "    source $WS/install/setup.bash"
    fi
}
