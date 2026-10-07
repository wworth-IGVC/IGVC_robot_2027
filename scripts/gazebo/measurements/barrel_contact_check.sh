#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Can the simulator report a barrel touch at all?
#
# autonomy_check.sh reports barrel contacts, and "none" is both the result
# everyone wants and the result a broken chain gives (no sensor in the world,
# no bridge entry, a logger listening on the wrong name). This is the test
# that the chain works, end to end, through ROS exactly as the check reads it:
# barrel contact sensor -> gz topic -> ros_gz_bridge -> contact_logger.py.
#
#   HIT    spawn 1.2 m from barrel 0's centre, facing it (0.371 m gap to the
#          chassis front), drive 0.3 m/s for 5 s. MUST report barrel_0.
#   CLEAR  spawn at the course start, drive the same command down the open
#          lane. MUST report no contact.
#
# PASS only if both hold and the bridge published the topic in both arms.
# First run 2026-10-01: HIT reported barrel_0 by the chassis (the robot stopped
# with its front face on the barrel's surface), CLEAR reported none.
#
# Docker: run INSIDE the Gazebo container. Starts and stops its own simulator;
# refuses to run on top of another one.
# ---------------------------------------------------------------------------
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/../igvc_env.sh" || exit 1
. "$HERE/../sim_preflight.sh"

echo "--- building ---"
igvc_build || exit 1

TRACK="$REPO/IGVC_track_generator/track_points.json"
# Barrel 0 from the track file, so the spawn follows the course if it changes.
read -r BX BY BR < <(python3 -c "
import json
o = json.load(open('$TRACK', encoding='utf-8'))['obstacles_m'][0]
print(o['x_m'], o['y_m'], o['radius_m'])
")
HIT_X=$(python3 -c "print('%.4f' % ($BX - 1.2))")

run_arm () {   # name, then launch arguments
    local name="$1"; shift
    local out="/tmp/barrel_contact_$name"
    rm -rf "$out"; mkdir -p "$out"
    FORCE=1 sim_preflight || return 1
    ros2 launch igvc_test_bringup gazebo_sim.launch.py headless:=true "$@" \
        > "$out/launch.log" 2>&1 &
    if ! python3 "$HERE/../wait_ready.py" odom 180 > "$out/ready.txt" 2>&1; then
        echo "  $name: simulator not ready, see $out/ready.txt"
        sim_teardown
        return 1
    fi
    python3 "$HERE/../contact_logger.py" "$out/contacts.txt" > "$out/logger.out" 2>&1 &
    local cl=$!
    sleep 2
    timeout 5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.3}}" > /dev/null 2>&1
    # Gazebo's DiffDrive holds the last command, so stop it explicitly.
    timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" \
        > /dev/null 2>&1
    sleep 2
    kill -INT "$cl" 2>/dev/null
    wait "$cl" 2>/dev/null
    timeout 60 gz model -m igvc_robot -p > "$out/final_pose.txt" 2>/dev/null
    sim_teardown
    local pubs touched
    pubs=$(awk '/^# publishers_seen/ {print $3}' "$out/contacts.txt")
    touched=$(grep -v '^#' "$out/contacts.txt" | awk '{print $2}' | sort -u | tr '\n' ' ')
    echo "$name publishers=${pubs:-0} touched=[${touched% }]" | tee "$out/summary.txt"
}

echo "barrel 0 at ($BX, $BY), radius $BR m"
run_arm HIT spawn_x:="$HIT_X" spawn_y:="$BY" spawn_yaw:=0.0
run_arm CLEAR

HIT=$(cat /tmp/barrel_contact_HIT/summary.txt 2>/dev/null)
CLEAR=$(cat /tmp/barrel_contact_CLEAR/summary.txt 2>/dev/null)
ok=1
case "$HIT" in
    *"publishers=0"*|"") echo "HIT   : FAIL  the bridge never published the topic"; ok=0 ;;
    *"barrel_0"*)        echo "HIT   : PASS  a touch was reported" ;;
    *)                   echo "HIT   : FAIL  drove into barrel 0 and nothing was reported"; ok=0 ;;
esac
case "$CLEAR" in
    *"publishers=0"*|"") echo "CLEAR : FAIL  the bridge never published the topic"; ok=0 ;;
    *"touched=[]"*)      echo "CLEAR : PASS  no contact in open lane" ;;
    *)                   echo "CLEAR : FAIL  a contact reported with no barrel near"; ok=0 ;;
esac
[ "$ok" = 1 ] && echo "BARREL CONTACT CHECK: PASS" || echo "BARREL CONTACT CHECK: FAIL"
[ "$ok" = 1 ]
