#!/usr/bin/env bash
# Do GPS waypoints land in the right place whatever the robot's start heading?
# One fresh simulator per heading, odom_axes:=robot (odometry axes follow the
# start heading, as on the real robot), robot at the course centre, then
# gps_heading_check.py. Run INSIDE the Gazebo container, one simulator.
#   bash scripts/gazebo/measurements/gps_heading_check.sh [yaw_deg ...]
# Default headings: 0 (east), 90, 180 and 135 (the course's own start).
# Exit 0 only if every heading passes. Negative controls: see the .py.
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/../igvc_env.sh" || exit 1
. "$HERE/../sim_preflight.sh"
FORCE=1 sim_preflight || exit 1
igvc_build || exit 1

YAWS="${*:-0 90 180 135}"
BASE_OUT="${GPS_CHECK_OUT:-/tmp/gps_heading_check}"
FAILS=0
for deg in $YAWS; do
    rad=$(python3 -c "import math; print(math.radians($deg))")
    echo "=== start heading $deg deg"
    ros2 launch igvc_test_bringup gazebo_sim.launch.py headless:=true rviz:=false \
        odom_axes:=robot spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=$rad \
        > "/tmp/gps_heading_sim_$deg.log" 2>&1 &
    LP=$!
    python3 "$HERE/../wait_ready.py" odom 240 >/dev/null 2>&1 || echo "WARN: wait_ready timed out"
    sleep 6
    SPAWN_X=0.0 SPAWN_Y=0.0 SPAWN_YAW=$rad GPS_CHECK_OUT="$BASE_OUT/yaw_$deg" \
        python3 "$HERE/gps_heading_check.py" || FAILS=$((FAILS + 1))
    kill -INT "$LP" 2>/dev/null; sleep 8; kill -TERM "$LP" 2>/dev/null; wait "$LP" 2>/dev/null
    sim_teardown
    sleep 3
done
echo
echo "headings failed: $FAILS of $(echo $YAWS | wc -w)"
[ "$FAILS" -eq 0 ]
