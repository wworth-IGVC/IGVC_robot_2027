#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One command to bring up the simulation.
#
#   Docker (Windows, Linux):  run it INSIDE the Gazebo container, see below.
#   pixi   (macOS, native):   pixi run sim     from the repo root.
#
# Builds the three packages the launch file needs, sources the workspace, and
# starts Gazebo, the robot, the ros_gz bridge and RViz.
#
#   RVIZ=0        skip RViz
#   HEADLESS=1    no Gazebo window either; for automated checks
#   CAMERAS=all   all three cameras instead of just the front one
#                 (robot 2026 only: the 2027 robot has one camera)
#   ROBOT=2026    the 2026 robot; the default is the launch file's, 2027
#   DRIVE=diffdrive  Gazebo's DiffDrive instead of ros2_control (the default
#                 under Docker; under pixi the default is diffdrive, see
#                 igvc_env.sh)
#   NAV=1         also start the navigation inputs: the ground-truth lanes
#                 (track_ground_truth_node, gt_nav_bridge_node, localization),
#                 i.e. gazebo_nav_test.launch.py instead of gazebo_sim
#
# The drive and the barrels are the launch files' defaults. Since 2026-10-16:
# drive:=ros2_control (gz_ros2_control and the real diff_drive_controller,
# which stops the robot 0.3 s after the last command; /odom from its wheel
# odometry) and, with NAV=1, obstacle_source:=lidar_map (the barrels from the
# simulated lidar). DRIVE=diffdrive gives the previous drive; for
# obstacle_source:=gt run ros2 launch by hand. The pixi path has no
# twist_stamper, so under pixi this script passes drive:=diffdrive by itself
# (IGVC_DEFAULT_DRIVE in igvc_env.sh).
#
# TO DRIVE IT you need a second shell in the SAME container, which is the one
# awkward part of the workflow and is worth reading before the meeting.
#
# `docker compose run` makes a NEW container with a random name every time, so
# a second `run` gives you a second, separate simulator. Use `up` instead,
# which honours container_name: igvc_gazebo, then exec into it:
#
#   # terminal 1, from a WSL2 shell
#   cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
#   docker compose -f docker-compose.windows.yml up -d igvc_gazebo
#   docker exec -it igvc_gazebo bash scripts/gazebo/start_sim.sh
#
#   # terminal 2, from another WSL2 shell
#   docker exec -it igvc_gazebo bash
#   source /root/ros2_ws/install/setup.bash
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
#
# WSL2, not PowerShell, or there is no window. See docs/GAZEBO_SETUP.md 3A.
# ---------------------------------------------------------------------------
set -o pipefail
. "$(dirname "$0")/igvc_env.sh" || exit 1

. "$(dirname "$0")/sim_preflight.sh"
sim_preflight || exit 1

RVIZ="${RVIZ:-1}"
HEADLESS="${HEADLESS:-0}"
CAMERAS="${CAMERAS:-front}"

echo "=============================================================="
echo " IGVC Gazebo simulation"
echo "=============================================================="

echo "--- building the five packages the bringup needs ---"
igvc_build || exit 1
echo "    ok"

# Regenerate the world if the track data is newer than the world file, so a
# change to track_points.json cannot silently leave a stale course behind.
WORLD="$REPO/src/igvc_test_description/worlds/igvc_course.sdf"
TRACK="$REPO/IGVC_track_generator/track_points.json"
if [ -f "$TRACK" ] && { [ ! -f "$WORLD" ] || [ "$TRACK" -nt "$WORLD" ]; }; then
    echo "--- track data is newer than the world; regenerating ---"
    python3 "$REPO/scripts/gazebo/generate_igvc_world.py" || exit 1
    igvc_build igvc_test_description || exit 1
fi

echo
echo "  drive it from a SECOND shell:"
igvc_shell_hint
echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo
if [ "$IGVC_RUNTIME" = "docker" ]; then
    echo "  if that container name does not exist, you started this with"
    echo "  'compose run' instead of 'compose up -d'. See the header of this file."
    echo
fi

LAUNCH_FILE="gazebo_sim.launch.py"
[ "${NAV:-0}" = "1" ] && LAUNCH_FILE="gazebo_nav_test.launch.py"

ARGS="sim_cameras:=$CAMERAS"
[ "$RVIZ" = "1" ] && ARGS="$ARGS rviz:=true"
[ "$HEADLESS" = "1" ] && ARGS="$ARGS headless:=true rviz:=false"
[ -n "${ROBOT:-}" ] && ARGS="$ARGS robot:=$ROBOT"
DRIVE="${DRIVE:-${IGVC_DEFAULT_DRIVE:-}}"
[ -n "$DRIVE" ] && ARGS="$ARGS drive:=$DRIVE"

echo "--- ros2 launch igvc_test_bringup $LAUNCH_FILE $ARGS ---"
exec ros2 launch igvc_test_bringup "$LAUNCH_FILE" $ARGS
