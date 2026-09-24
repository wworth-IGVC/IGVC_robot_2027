#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Does the Gazebo bringup produce a robot that DRIVES, publishes the topic
# names the IGVC stack consumes, and agrees with the simulator about where it
# is - and do real 2026 nodes run against it?
#
# "gz sim started" and "the topics exist" both pass on a badly broken setup.
# A world with no robot in it still publishes /clock. A robot with a wrong
# wheel radius still publishes /odom. A bridge that renamed every topic
# correctly still passes every name check while reporting a position rotated
# 135 degrees away from the truth. So each part below is chosen to fail:
#
#   PART 1  every contract topic carries a MESSAGE, not just a name
#   PART 2  commanding a velocity moves the odometry
#   PART 3  odometry agrees with the simulator's own ground truth, in both
#           direction and distance. This is the one that catches a wrong
#           odom frame and a wrong wheel radius, and it is new.
#   PART 4  real igvc_lane_detection nodes consume it and produce a costmap
#
# Docker: run this INSIDE the Gazebo container. pixi: `pixi run smoke`.
#
#   RVIZ=1        also start RViz and check the node comes up (needs a display)
#   STARTUP_LIMIT=300  ceiling, in wall seconds, on waiting for the stack to
#                 come up. It is a ceiling, not a wait: the script proceeds
#                 the moment Gazebo, the bridge and odometry are live, so a
#                 fast machine does not pay for a slow one.
#   NAV=0         skip PART 4 and launch the simulator alone
#   CAMERAS=all   exercise all three cameras instead of just the front one
# ---------------------------------------------------------------------------
set -o pipefail
. "$(dirname "$0")/igvc_env.sh" || exit 1

. "$(dirname "$0")/sim_preflight.sh"
FORCE=1 sim_preflight || exit 1

NAV="${NAV:-1}"
CAMERAS="${CAMERAS:-front}"

# test_robot.urdf.xacro resolves its includes with
# $(find igvc_test_description), so that package has to be in the ament index.
# igvc_test_bringup carries the launch files and gazebo_odom_shim;
# igvc_lane_detection carries the ground-truth nodes PART 4 runs. All are
# install-only, so this is seconds rather than a real build.
echo "--- building the four packages the bringup needs ---"
igvc_build || exit 1
echo "  installed at: $(ros2 pkg prefix igvc_test_description 2>/dev/null)"

echo
echo "=============================================================="
echo " Gazebo IGVC bringup smoke test"
echo "=============================================================="

# RVIZ=1 additionally starts RViz and checks the node comes up. That needs
# a display, so it only works from a WSL2 shell, never from PowerShell.
LAUNCH_FILE="gazebo_nav_test.launch.py"
[ "$NAV" = "0" ] && LAUNCH_FILE="gazebo_sim.launch.py"

LAUNCH_ARGS="headless:=true sim_cameras:=$CAMERAS"
if [ "${RVIZ:-0}" = "1" ]; then
    LAUNCH_ARGS="headless:=false rviz:=true sim_cameras:=$CAMERAS"
fi

# nav2:=false is NOT optional. PART 2 and 3 command a velocity and measure
# where the robot ends up, and Nav2 publishes to the same /cmd_vel. With both
# driving, a run on 2026-09-15 found the robot already 21 m from spawn before
# the test had commanded anything, "coasted" 9.6 m after the burst ended, and
# disagreed with ground truth by 4.62 degrees - every one of those numbers was
# Nav2, none of them the property under test. Nav2 belongs in
# autonomy_check.sh, which measures it properly by never touching /cmd_vel.
# PART 4's ground-truth nodes do not need Nav2 and still run.
[ "$NAV" = "0" ] || LAUNCH_ARGS="$LAUNCH_ARGS nav2:=false"

echo "launch file: $LAUNCH_FILE"
echo "launch args: $LAUNCH_ARGS"

ros2 launch igvc_test_bringup "$LAUNCH_FILE" $LAUNCH_ARGS \
    > /tmp/bringup.log 2>&1 &
LAUNCH_PID=$!

# Wait for the stack to be READY, not for a fixed number of seconds.
#
# This was `sleep 35` until 2026-09-24. A fixed wall-clock wait is wrong in
# both directions: wasted time on a fast machine, and on a slow one (tier C,
# or a Mac) a stack that is not up yet. wait_ready.py returns the moment
# /clock is advancing and /odom is publishing, which proves Gazebo, the
# robot, DiffDrive, the bridge and gazebo_odom_shim are all live. The
# ground-truth nodes PART 4 needs come up with them and PART 4's own topic
# timeouts cover the last few seconds.
STARTUP_LIMIT="${STARTUP_LIMIT:-300}"
echo "waiting for the stack to come up (ceiling ${STARTUP_LIMIT}s)..."
python3 "$(dirname "$0")/wait_ready.py" odom "$STARTUP_LIMIT"

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
        printf "  %-58s PASS\n" "$T"
        PASS=$((PASS + 1))
    else
        printf "  %-58s FAIL\n" "$T"
        FAIL=$((FAIL + 1))
    fi
}

echo
echo "=============================================================="
echo " PART 1: the section 3.7 interface contract"
echo "=============================================================="
echo "These are the names the 2026 stack was written against. They are"
echo "produced by config/gazebo_bridge.yaml renaming Gazebo's own names."
echo
check_topic /clock             rosgraph_msgs/msg/Clock
check_topic /scan              sensor_msgs/msg/LaserScan
check_topic /tf                tf2_msgs/msg/TFMessage
check_topic /tf_static         tf2_msgs/msg/TFMessage
check_topic /joint_states      sensor_msgs/msg/JointState
check_topic /isaac_joint_state sensor_msgs/msg/JointState
check_topic /odom              nav_msgs/msg/Odometry
check_topic /front_zed_camera_x/zed_node/odom nav_msgs/msg/Odometry
check_topic /imu               sensor_msgs/msg/Imu

# Only the cameras that sim_cameras actually created. Every camera in the
# contract is bridged unconditionally, so left and right are LISTED above even
# under sim_cameras:=front - which is exactly why this loop checks for
# messages and not for names.
CAM_LIST="front"
[ "$CAMERAS" = "all" ] && CAM_LIST="front left right"
[ "$CAMERAS" = "none" ] && CAM_LIST=""
for C in $CAM_LIST; do
    N="${C}_zed_camera_x"
    check_topic "/$N/zed_node/rgb/color/rect/image"       sensor_msgs/msg/Image       25
    check_topic "/$N/zed_node/rgb/color/rect/camera_info" sensor_msgs/msg/CameraInfo  25
    check_topic "/$N/zed_node/depth/depth_registered"     sensor_msgs/msg/Image       25
    check_topic "/$N/zed_node/point_cloud/cloud_registered" sensor_msgs/msg/PointCloud2 25
done

# ── does the frame id on the images actually EXIST? ─────────────────────────
#
# This check exists because the answer was no, silently, until 2026-09-15.
# gazebo_sim.urdf.xacro stamped *_left_camera_optical_frame while the ZED
# macro creates *_left_camera_frame_optical. Every check above passed anyway:
# the images flowed, the frame id string looked exactly right, and `ros2 topic
# echo` showed what you would expect. Only a TF lookup can tell, and without
# one igvc_lane_detection's projection_utils cannot turn a lane pixel into a
# ground point - lane detection that looks like it works and produces nothing.
#
# Checking the string against the xacro is not a test. Resolving it is.
echo
echo "--- do the camera frame ids resolve in TF? ---"
for C in $CAM_LIST; do
    N="${C}_zed_camera_x"
    FID=$(timeout 25 ros2 topic echo --once --field header.frame_id \
          "/$N/zed_node/rgb/color/rect/image" 2>/dev/null | grep -v '^\[' | head -1)
    if [ -z "$FID" ]; then
        printf "  %-58s FAIL\n" "$N frame id (no image)"
        FAIL=$((FAIL + 1))
        continue
    fi
    # Capture first, match second. Do NOT pipe tf2_echo straight into
    # `grep -q` under `set -o pipefail`, which this script sets:
    #
    #   tf2_echo prints forever until the timeout. grep -q matches, exits
    #   immediately, and tf2_echo takes SIGPIPE. pipefail then reports the
    #   pipeline as 141 - so a SUCCESSFUL lookup reads as a failure.
    #
    # That cost a round of "the frame is missing from TF" when the frame was
    # present and correct the whole time, verified by hand with the identical
    # command outside a pipeline. Demonstrated:
    #   set -o pipefail
    #   (echo a; sleep 3; echo Translation:; sleep 3) | grep -q Translation:
    #   -> 141
    #
    # 25 s rather than 12 for a second reason: `ros2 run` has to discover the
    # node graph first, and a timeout is not evidence of absence either.
    TFOUT=$(timeout 25 ros2 run tf2_ros tf2_echo base_link "$FID" 2>&1 | head -20)
    if printf '%s\n' "$TFOUT" | grep -q "Translation:"; then
        printf "  %-58s PASS\n" "base_link -> $FID"
        PASS=$((PASS + 1))
    else
        printf "  %-58s FAIL\n" "base_link -> $FID"
        echo "      that frame id is stamped on the images but is not in the"
        echo "      TF tree. Check gz_frame_id in gazebo_sim.urdf.xacro"
        echo "      against the link names in zed_macro.urdf.xacro."
        FAIL=$((FAIL + 1))
    fi
done

if [ "${RVIZ:-0}" = "1" ]; then
    echo
    echo "--- RViz ---"
    # Captured first, for the same pipefail reason as the TF check above.
    NODES=$(ros2 node list 2>/dev/null)
    if echo "$NODES" | grep -q rviz2; then
        printf "  %-58s PASS\n" "rviz2 node running"
        PASS=$((PASS + 1))
    else
        printf "  %-58s FAIL\n" "rviz2 node running"
        FAIL=$((FAIL + 1))
    fi
fi

# ── helpers for the pose comparisons ────────────────────────────────────────
odom_xy () {
    timeout 15 ros2 topic echo --once /odom nav_msgs/msg/Odometry \
        2>/dev/null > /tmp/smoke_odom.yaml
    python3 -c "
import re
t = open('/tmp/smoke_odom.yaml').read()
def g(k):
    m = re.search(r'position:[\s\S]*?' + k + r':\s*(-?[\d.eE+-]+)', t)
    return float(m.group(1)) if m else 0.0
print('%.6f %.6f' % (g('x'), g('y')))
" 2>/dev/null
}

# x y z. The z matters: the ground slab is finite, and a robot that has driven
# off the edge keeps its wheels turning in mid-air, so odometry keeps counting
# while the robot falls. Any measurement taken then is meaningless.
world_xyz () {
    gz model -m igvc_robot -p 2>/dev/null > /tmp/smoke_world.txt
    python3 -c "
import re
t = open('/tmp/smoke_world.txt').read()
m = re.search(r'\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\]', t)
print('%.6f %.6f %.4f' % (float(m.group(1)), float(m.group(2)), float(m.group(3)))
      if m else '0 0 0')
" 2>/dev/null
}

on_ground () {   # $1 = "x y z" triple
    python3 -c "
import sys
z = float('$1'.split()[2])
sys.exit(0 if abs(z - 0.2311) < 0.10 else 1)
" 2>/dev/null
}

echo
echo "=============================================================="
echo " PART 2 and 3: does it drive, and does it know where it is?"
echo "=============================================================="

# DRIVE SLOWLY AND BRIEFLY, AND ON SIMULATION TIME.
#
# The ground is a finite 39.54 x 33.62 m slab and the robot spawns 9.44 m from
# its -x edge on the spawn heading of 134.87 deg. Gazebo's DiffDrive holds the
# last /cmd_vel it received, so the robot keeps driving until something tells
# it to stop.
#
# The previous version of this part published the burst with `ros2 topic pub`
# and then took four pose samples with `ros2 topic echo --once` and
# `gz model -p` BEFORE publishing the stop. Each CLI call spends seconds just
# starting, and the robot drove on the held command the whole time, so the
# excursion depended on how fast the machine starts a Python process rather
# than on anything in this script. Measured 2026-09-24 on the reference
# laptop: a "4 s" coast window that really spanned about 11.7 s, 9.5 m of
# travel against 9.44 m to the edge, and a robot that drove off the slab. On a
# slower machine, a Mac under emulation in particular, it could only be worse.
#
# drive_probe.py runs burst, coast and stop inside ONE process on simulation
# time, so the excursion is bounded by arithmetic: at most 0.4 m/s x (3 s
# burst + 4 s coast + 1 s stop) = 3.2 m, on any machine at any speed. The
# pose samples that are slow to take are taken only at rest, before and
# after, where their latency cannot move the robot.
O_BEFORE=$(odom_xy)
W_BEFORE=$(world_xyz)
echo "  odom  before : $O_BEFORE"
echo "  world before : $W_BEFORE"
echo

python3 "$(dirname "$0")/drive_probe.py" > /tmp/smoke_probe.txt 2>&1
PROBE_RC=$?
grep -v '^PROBE_' /tmp/smoke_probe.txt
probe () { sed -n "s/^$1=//p" /tmp/smoke_probe.txt | tail -1; }
PROBE_RESULT=$(probe PROBE_RESULT)
if [ "$PROBE_RESULT" = "HARNESS_FAIL" ] || [ -z "$PROBE_RESULT" ]; then
    echo "  FAIL: the drive probe could not run (exit $PROBE_RC). Its output:"
    sed 's/^/    /' /tmp/smoke_probe.txt | tail -20
    FAIL=$((FAIL + 1))
fi

# ── the /cmd_vel timeout question ───────────────────────────────────────────
#
# docs/GAZEBO_SETUP.md 8.6 left this open and three attempts failed to settle
# it, because the coast was measured between CLI samples taken an unknown
# number of seconds apart. The probe measures it between two odometry
# messages whose stamps it records, starting at the last command it actually
# sent. diff_drive_controller on the real robot times out and stops.
#
# Whether the wheels stayed on the ground during the coast is settled by the
# on-ground check on the final pose below: the path is a straight line of at
# most 3.2 m, so a robot that ends on the slab never left it.
echo
if [ "$(probe PROBE_COAST_HELD)" = "yes" ]; then
    echo "  /cmd_vel timeout: DiffDrive HELD the last command, $(probe PROBE_COAST_M) m"
    echo "  in $(probe PROBE_COAST_WINDOW_S) s of sim time with no command at all."
    echo "  diff_drive_controller on the real robot times out instead, so sim"
    echo "  is the less safe of the two."
elif [ "$(probe PROBE_COAST_HELD)" = "no" ]; then
    echo "  /cmd_vel timeout: the robot stopped on its own, $(probe PROBE_COAST_M) m in"
    echo "  $(probe PROBE_COAST_WINDOW_S) s of sim time after the last command."
fi

O_AFTER=$(odom_xy)
W_AFTER=$(world_xyz)
echo
echo "  odom  after  : $O_AFTER"
echo "  world after  : $W_AFTER"

if ! on_ground "$W_AFTER"; then
    echo
    echo "  FAIL: the robot is not on the ground slab. Everything below this"
    echo "        line is measured from a falling robot and means nothing."
    FAIL=$((FAIL + 1))
fi

RESULT=$(python3 -c "
import math, sys
ob = [float(v) for v in '$O_BEFORE'.split()]
oa = [float(v) for v in '$O_AFTER'.split()]
wb = [float(v) for v in '$W_BEFORE'.split()]
wa = [float(v) for v in '$W_AFTER'.split()]
do = (oa[0] - ob[0], oa[1] - ob[1])
dw = (wa[0] - wb[0], wa[1] - wb[1])
no = math.hypot(*do)
nw = math.hypot(*dw)
if no < 1e-6 or nw < 1e-6:
    print('%.3f %.3f 999.0 0.0 DRIVE_FAIL' % (no, nw)); sys.exit()
dot = (do[0]*dw[0] + do[1]*dw[1]) / (no*nw)
ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
ratio = no / nw
verdict = 'OK'
if ang > 5.0:      verdict = 'FRAME_FAIL'
elif ratio < 0.85 or ratio > 1.15: verdict = 'SCALE_FAIL'
print('%.3f %.3f %.2f %.4f %s' % (no, nw, ang, ratio, verdict))
")
set -- $RESULT
ODIST="$1"; WDIST="$2"; ANGLE="$3"; RATIO="$4"; VERDICT="$5"

echo
echo "  odometry displacement : $ODIST m"
echo "  simulator ground truth: $WDIST m"
echo "  heading disagreement  : $ANGLE deg     (a wrong odom frame shows up here)"
echo "  distance ratio        : $RATIO         (a wrong wheel radius shows up here)"

DRIVE=1
if python3 -c "import sys; sys.exit(0 if float('$ODIST') > 0.25 else 1)"; then
    DRIVE=0
fi

FRAME=1
[ "$VERDICT" = "OK" ] && FRAME=0

echo
if [ "$DRIVE" -eq 0 ]; then
    echo "  DRIVE TEST : PASS   robot moved $ODIST m on /cmd_vel"
else
    echo "  DRIVE TEST : FAIL   moved $ODIST m, expected more than 0.25"
fi
case "$VERDICT" in
    OK)         echo "  FRAME TEST : PASS   odometry agrees with the simulator" ;;
    FRAME_FAIL) echo "  FRAME TEST : FAIL   odometry points $ANGLE deg away from"
                echo "                      where the robot actually went. The"
                echo "                      spawn yaw is almost certainly not"
                echo "                      reaching gazebo_odom_shim." ;;
    SCALE_FAIL) echo "  FRAME TEST : FAIL   odometry distance is ${RATIO}x the truth."
                echo "                      Suspect wheel_radius or"
                echo "                      wheel_separation in gazebo_sim.urdf.xacro." ;;
    *)          echo "  FRAME TEST : FAIL   robot did not move; nothing to compare" ;;
esac

# ── PART 4 ──────────────────────────────────────────────────────────────────
GT=0
if [ "$NAV" = "1" ]; then
    echo
    echo "=============================================================="
    echo " PART 4: do real 2026 nodes run against Gazebo?"
    echo "=============================================================="
    echo "track_ground_truth_node reads track_points.json and subscribes to"
    echo "nothing, so it is simulator-independent. gt_nav_bridge_node is the"
    echo "one that matters: it only publishes /lane_costmap once it has BOTH"
    echo "the ground-truth grid AND simulator odometry, and it is remapped"
    echo "onto the ZED contract topic, so it fails if the rename is wrong."
    echo
    check_topic /lane_ground_truth nav_msgs/msg/OccupancyGrid 20
    check_topic /lane_map          nav_msgs/msg/OccupancyGrid 20
    check_topic /lane_costmap      nav_msgs/msg/OccupancyGrid 20

    # The local costmap is a base_link-frame crop of the fixed grid, so its
    # ORIGIN never moves but its CONTENTS must change as the robot drives.
    # A frozen costmap means odometry is not reaching the node.
    timeout 20 ros2 topic echo --once /lane_costmap nav_msgs/msg/OccupancyGrid \
        2>/dev/null | md5sum > /tmp/cm_a.txt
    timeout 5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.6}, angular: {z: 0.0}}' > /dev/null 2>&1
    # DiffDrive holds the last command, so stop it explicitly or the robot
    # drives off the course while the rest of the test runs.
    timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0}, angular: {z: 0.0}}' > /dev/null 2>&1
    sleep 2
    timeout 20 ros2 topic echo --once /lane_costmap nav_msgs/msg/OccupancyGrid \
        2>/dev/null | md5sum > /tmp/cm_b.txt

    if [ -s /tmp/cm_a.txt ] && [ -s /tmp/cm_b.txt ] \
       && ! cmp -s /tmp/cm_a.txt /tmp/cm_b.txt; then
        printf "  %-58s PASS\n" "/lane_costmap changed as the robot drove"
        PASS=$((PASS + 1))
    else
        printf "  %-58s FAIL\n" "/lane_costmap changed as the robot drove"
        FAIL=$((FAIL + 1))
        GT=1
    fi
fi

echo
echo "=============================================================="
echo " topic checks passed: $PASS   failed: $FAIL"
echo "=============================================================="

kill "$LAUNCH_PID" 2>/dev/null
sleep 3
pkill -f 'gz sim' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null

if [ "$FAIL" -ne 0 ] || [ "$DRIVE" -ne 0 ] || [ "$FRAME" -ne 0 ] || [ "$GT" -ne 0 ]; then
    echo
    echo "--- launch log tail, for diagnosis ---"
    tail -45 /tmp/bringup.log
    exit 1
fi
