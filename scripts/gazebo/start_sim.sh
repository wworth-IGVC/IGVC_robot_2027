#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One command to bring up the simulation. Run this INSIDE the Gazebo container.
#
# Builds the three packages the launch file needs, sources the workspace, and
# starts Gazebo, the robot, the ros_gz bridge and RViz.
#
#   RVIZ=0        skip RViz
#   HEADLESS=1    no Gazebo window either; for automated checks
#   CAMERAS=all   all three cameras instead of just the front one
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
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

REPO=/root/ros2_ws/src/IGVC_robot_2026
RVIZ="${RVIZ:-1}"
HEADLESS="${HEADLESS:-0}"
CAMERAS="${CAMERAS:-front}"

echo "=============================================================="
echo " IGVC Gazebo simulation"
echo "=============================================================="

echo "--- building igvc_test_description, igvc_test_bringup, zed_description ---"
cd /root/ros2_ws || exit 1
colcon build --symlink-install \
    --base-paths "$REPO/src" \
    --packages-select zed_description igvc_test_description igvc_test_bringup \
    > /tmp/colcon.log 2>&1
RC=$?
if [ "$RC" -ne 0 ]; then
    echo "FAIL: colcon build exited $RC"
    tail -30 /tmp/colcon.log
    exit 1
fi
source /root/ros2_ws/install/setup.bash
echo "    ok"

# Regenerate the world if the track data is newer than the world file, so a
# change to track_points.json cannot silently leave a stale course behind.
WORLD="$REPO/src/igvc_test_description/worlds/igvc_course.sdf"
TRACK="$REPO/IGVC_track_generator/track_points.json"
if [ -f "$TRACK" ] && { [ ! -f "$WORLD" ] || [ "$TRACK" -nt "$WORLD" ]; }; then
    echo "--- track data is newer than the world; regenerating ---"
    python3 "$REPO/scripts/gazebo/generate_igvc_world.py" || exit 1
    colcon build --symlink-install --base-paths "$REPO/src" \
        --packages-select igvc_test_description > /dev/null 2>&1
    source /root/ros2_ws/install/setup.bash
fi

echo
echo "  drive it from a SECOND shell in this same container:"
echo "    docker exec -it igvc_gazebo bash"
echo "    source /root/ros2_ws/install/setup.bash"
echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo
echo "  if that container name does not exist, you started this with"
echo "  'compose run' instead of 'compose up -d'. See the header of this file."
echo

ARGS="sim_cameras:=$CAMERAS"
[ "$RVIZ" = "1" ] && ARGS="$ARGS rviz:=true"
[ "$HEADLESS" = "1" ] && ARGS="$ARGS headless:=true rviz:=false"

echo "--- ros2 launch igvc_test_bringup gazebo_sim.launch.py $ARGS ---"
exec ros2 launch igvc_test_bringup gazebo_sim.launch.py $ARGS
