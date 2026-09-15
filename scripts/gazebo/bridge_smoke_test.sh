#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Does ros_gz actually carry sensor data from Gazebo into ROS 2?
#
# render_check.sh proves Gazebo renders on the GPU. That is necessary and not
# sufficient: the whole point of the Harmonic pairing is that ROS 2 nodes see
# the data. This runs the real bridge and checks that a ROS 2 subscriber
# receives an Image, a LaserScan and a clock, which is the minimum subset of
# the section 3.7 interface contract that Gazebo can satisfy on its own.
#
# It does NOT test the ZED namespace. That needs the RQ-03 shim, which does not
# exist yet.
# ---------------------------------------------------------------------------
set -o pipefail

source /opt/ros/humble/setup.bash

WORLD="${WORLD:-$(dirname "$0")/render_check.sdf}"
PASS=0
FAIL=0

check () {
    # check <label> <ros topic> <ros type> <timeout seconds>
    local LABEL="$1" TOPIC="$2" TYPE="$3" T="${4:-25}"
    local OUT
    OUT=$(timeout "$T" ros2 topic echo --once "$TOPIC" "$TYPE" 2>&1)
    if [ -n "$OUT" ] && ! echo "$OUT" | grep -qi "does not appear\|Failed\|timeout"; then
        printf " %-34s PASS\n" "$LABEL"
        PASS=$((PASS + 1))
    else
        printf " %-34s FAIL\n" "$LABEL"
        FAIL=$((FAIL + 1))
    fi
}

echo "=============================================================="
echo " ros_gz bridge smoke test"
echo "=============================================================="
echo "world: $WORLD"
echo

gz sim -v 2 -s -r --headless-rendering "$WORLD" > /tmp/bridge_gzsim.log 2>&1 &
GZ_PID=$!
sleep 12

if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "FAIL: gz sim died during startup"
    tail -20 /tmp/bridge_gzsim.log
    exit 1
fi

# One bridge process, three mappings. The gz type names are Harmonic's.
ros2 run ros_gz_bridge parameter_bridge \
    /render_check/camera@sensor_msgs/msg/Image[gz.msgs.Image \
    /render_check/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 8

echo "--- ROS 2 topics visible ---"
ros2 topic list 2>/dev/null | grep -E "render_check|clock"
echo

check "camera  -> sensor_msgs/Image"      /render_check/camera sensor_msgs/msg/Image
check "lidar   -> sensor_msgs/LaserScan"  /render_check/scan   sensor_msgs/msg/LaserScan
check "clock   -> rosgraph_msgs/Clock"    /clock               rosgraph_msgs/msg/Clock

kill "$BRIDGE_PID" 2>/dev/null
kill "$GZ_PID" 2>/dev/null
wait "$BRIDGE_PID" 2>/dev/null
wait "$GZ_PID" 2>/dev/null

echo
echo "=============================================================="
echo " passed: $PASS   failed: $FAIL"
echo "=============================================================="
[ "$FAIL" -eq 0 ] || exit 1
