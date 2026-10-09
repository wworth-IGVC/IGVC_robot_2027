#!/usr/bin/env bash
# Does the GPS mission start by itself when the lane-following robot reaches
# the start waypoint? Starts the full simulator stack (Nav2 and the lane
# navigator on ground truth), runs gps_mission_check.py against the real
# mission_planner_node, tears everything down. Run INSIDE the Gazebo container,
# one simulator. Exit 0 only on PASS. Negative control: PLANNER_PYTHONPATH set
# to a copy of the old planner code must FAIL.
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/../igvc_env.sh" || exit 1
. "$HERE/../sim_preflight.sh"
FORCE=1 sim_preflight || exit 1

echo "--- building ---"
igvc_build || exit 1

ros2 launch igvc_test_bringup gazebo_nav_test.launch.py headless:=true \
    > /tmp/gps_mission_check_sim.log 2>&1 &
LP=$!
# Start the planner before the robot moves: it must find its own way to the
# start waypoint's gate, so it needs to be watching from the start line.
python3 "$HERE/../wait_ready.py" odom 240 || echo "WARN: wait_ready timed out"

python3 "$HERE/gps_mission_check.py"
RC=$?

kill -INT "$LP" 2>/dev/null
sleep 8
kill -TERM "$LP" 2>/dev/null
wait "$LP" 2>/dev/null
sim_teardown
exit $RC
