#!/usr/bin/env bash
# Does the GPS mission start when the robot reaches the start waypoint, and not
# before?  Starts the simulator (no Nav2), runs gps_start_check.py against the
# real mission_planner_node, tears everything down.  Run INSIDE the Gazebo
# container, one simulator, nothing else driving the robot.
#   bash scripts/gazebo/measurements/gps_start_check.sh
# Exit 0 only if all three arms pass.  See gps_start_check.py for the arms and
# the negative control (the code before 2026-10-08 must fail arm F).
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/../igvc_env.sh" || exit 1
. "$HERE/../sim_preflight.sh"
FORCE=1 sim_preflight || exit 1

echo "--- building ---"
igvc_build || exit 1

# Open ground at the course centre, facing east: the 8 m drive and the
# waypoints beyond it stay on the slab and clear of barrels. The course's own
# start faces the slab's edge.
export SPAWN_X=0.0 SPAWN_Y=0.0 SPAWN_YAW=0.0
ros2 launch igvc_test_bringup gazebo_sim.launch.py headless:=true rviz:=false \
    spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_yaw:=$SPAWN_YAW \
    > /tmp/gps_start_check_sim.log 2>&1 &
LP=$!
python3 "$HERE/../wait_ready.py" odom 240 || echo "WARN: wait_ready timed out"
sleep 8     # a few /fix messages at 1 Hz before the first arm

python3 "$HERE/gps_start_check.py"
RC=$?

kill -INT "$LP" 2>/dev/null
sleep 8
kill -TERM "$LP" 2>/dev/null
wait "$LP" 2>/dev/null
sim_teardown
exit $RC
