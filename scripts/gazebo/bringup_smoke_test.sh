#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Does the Gazebo bringup actually produce a robot that DRIVES?
#
# "gz sim started" and "the topics exist" both pass on a badly broken setup.
# A world with no robot in it still publishes /clock. A robot with a wrong
# wheel radius still publishes /odom. The test that can actually fail is:
# command a velocity, and watch the odometry move by roughly the right amount.
#
# Run this INSIDE the Gazebo container.
# ---------------------------------------------------------------------------
set -o pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

REPO=/root/ros2_ws/src/IGVC_robot_2026

# test_robot.urdf.xacro resolves its includes with
# $(find igvc_test_description), so that package has to be in the ament index.
# Both packages are install-only, so this is seconds rather than a real build.
echo "--- building igvc_test_description + igvc_test_bringup ---"
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
echo "  installed at: $(ros2 pkg prefix igvc_test_description 2>/dev/null)"

echo
echo "=============================================================="
echo " Gazebo IGVC bringup smoke test"
echo "=============================================================="

ros2 launch igvc_test_bringup gazebo_sim.launch.py headless:=true \
    > /tmp/bringup.log 2>&1 &
LAUNCH_PID=$!

echo "waiting for the stack to come up..."
sleep 35

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FAIL: launch died during startup. Last 40 lines:"
    tail -40 /tmp/bringup.log
    exit 1
fi

echo
echo "--- ROS 2 topics ---"
ros2 topic list 2>/dev/null | sort | sed 's/^/  /'

PASS=0
FAIL=0
check_topic () {
    local T="$1" TYPE="$2" TMO="${3:-15}"
    if timeout "$TMO" ros2 topic echo --once "$T" "$TYPE" >/dev/null 2>&1; then
        printf "  %-30s PASS\n" "$T"
        PASS=$((PASS + 1))
    else
        printf "  %-30s FAIL\n" "$T"
        FAIL=$((FAIL + 1))
    fi
}

echo
echo "--- is data actually flowing? ---"
check_topic /clock           rosgraph_msgs/msg/Clock
check_topic /odom            nav_msgs/msg/Odometry
check_topic /scan            sensor_msgs/msg/LaserScan
check_topic /imu             sensor_msgs/msg/Imu
check_topic /joint_states    sensor_msgs/msg/JointState
check_topic /tf              tf2_msgs/msg/TFMessage
check_topic /front_zed/image sensor_msgs/msg/Image 25

odom_x () {
    timeout 15 ros2 topic echo --once /odom nav_msgs/msg/Odometry 2>/dev/null \
        | grep -A3 'position:' | grep -m1 'x:' | tr -d ' ' | cut -d: -f2
}

echo
echo "--- THE REAL TEST: command a velocity, does odometry move? ---"
BEFORE=$(odom_x)
echo "  odom x before : ${BEFORE:-<none>}"

timeout 8 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.6}, angular: {z: 0.0}}' > /dev/null 2>&1

sleep 2
AFTER=$(odom_x)
echo "  odom x after  : ${AFTER:-<none>}"

MOVED=$(python3 -c "
try:
    print('%.3f' % abs(float('${AFTER:-0}') - float('${BEFORE:-0}')))
except Exception:
    print('0.0')
")
echo "  displacement  : $MOVED m  (8 s at 0.6 m/s, minus spin-up)"

DRIVE=1
if python3 -c "import sys; sys.exit(0 if float('$MOVED') > 0.25 else 1)"; then
    DRIVE=0
fi

echo
echo "=============================================================="
if [ "$DRIVE" -eq 0 ]; then
    echo " DRIVE TEST: PASS   robot moved $MOVED m on /cmd_vel"
else
    echo " DRIVE TEST: FAIL   moved $MOVED m, expected more than 0.25"
fi
echo " topic checks passed: $PASS   failed: $FAIL"
echo "=============================================================="

kill "$LAUNCH_PID" 2>/dev/null
sleep 3
pkill -f 'gz sim' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null

if [ "$FAIL" -ne 0 ] || [ "$DRIVE" -ne 0 ]; then
    echo
    echo "--- launch log tail, for diagnosis ---"
    tail -45 /tmp/bringup.log
    exit 1
fi
