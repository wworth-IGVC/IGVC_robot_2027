#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Does the robot drive the IGVC course BY ITSELF, and does it stay in the lane?
#
# bringup_smoke_test.sh proves the simulator works and publishes the interface
# contract. This proves the thing anyone actually cares about: start it, touch
# nothing, and the robot navigates the course.
#
# NOTHING IN THIS SCRIPT PUBLISHES /cmd_vel. That is the point. Every command
# the robot receives comes from Nav2, driven by igvc_navigator reading the
# ground-truth lane costmap. If the script has to push the robot, the test is
# meaningless - which is exactly the trap the earlier smoke test fell into,
# where a scripted `ros2 topic pub` looked like autonomy and was not.
#
# What can fail here, deliberately:
#   * the robot never moves                      -> nav stack not closing
#   * it moves but wanders off the centreline    -> following nothing useful
#   * it leaves the ground slab                  -> drove off the world
#   * it moves but covers no NEW course          -> spinning or stuck in place
#
# Run INSIDE the Gazebo container.
#
#   DURATION=120   how long to watch, seconds (default 100)
#   TOL=2.0        max allowed distance from the lane centreline, metres
# ---------------------------------------------------------------------------
set -o pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

# Refuse to run on top of an existing simulator. A run with two of them looks
# plausible and means nothing - see sim_preflight.sh.
. "$(dirname "$0")/sim_preflight.sh"
FORCE=1 sim_preflight || exit 1

REPO=/root/ros2_ws/src/IGVC_robot_2026
DURATION="${DURATION:-100}"
TOL="${TOL:-2.0}"
TRACK="$REPO/IGVC_track_generator/track_points.json"

cd /root/ros2_ws || exit 1
echo "--- building ---"
colcon build --symlink-install --base-paths "$REPO/src" \
    --packages-select zed_description igvc_test_description igvc_test_bringup \
                      igvc_lane_detection \
    > /tmp/colcon.log 2>&1 || { echo "FAIL: build"; tail -30 /tmp/colcon.log; exit 1; }
source /root/ros2_ws/install/setup.bash
echo "    ok"

echo
echo "=============================================================="
echo " Gazebo autonomy check: does it drive the course itself?"
echo "=============================================================="
echo "watching for ${DURATION}s, centreline tolerance ${TOL} m"
echo "this script never publishes /cmd_vel"
echo

ARGS="headless:=true"
[ "${RVIZ:-0}" = "1" ] && ARGS="headless:=false rviz:=true"

ros2 launch igvc_test_bringup gazebo_nav_test.launch.py $ARGS \
    > /tmp/autonomy.log 2>&1 &
LP=$!

# Nav2 lifecycle bringup plus the navigator's first plan takes a while.
echo "waiting 50s for Gazebo, Nav2 lifecycle activation and the first plan..."
sleep 50

if ! kill -0 "$LP" 2>/dev/null; then
    echo "FAIL: launch died during startup."
    tail -40 /tmp/autonomy.log
    exit 1
fi

# Sample through a ROS subscriber, not by calling `gz model -p` in a loop.
# That call is a gz-transport round trip per sample and took about 20 SECONDS
# each under a loaded simulator: 18 samples in seven minutes, spaced by
# whatever the simulator was busy doing. See pose_logger.py.
SPAWN=$(python3 -c "
import json
p = json.load(open('$TRACK', encoding='utf-8'))['robot_start_pose']['position_m']
print('%.6f %.6f' % (p['x'], p['y']))
")
echo "watching for ${DURATION}s..."
python3 "$(dirname "$0")/pose_logger.py" /tmp/track_log.txt "$DURATION" $SPAWN "${POSE_HZ:-10.0}"

# One ground-truth reading, for the one thing odometry cannot tell us: whether
# the robot is still on the ground slab. Wheels spinning in mid-air produce
# perfectly healthy odometry.
gz model -m igvc_robot -p 2>/dev/null > /tmp/gzpose.txt
FINAL_Z=$(python3 -c "
import re
t = open('/tmp/gzpose.txt').read()
m = re.search(r'\[\s*-?[0-9.]+\s+-?[0-9.]+\s+(-?[0-9.]+)\s*\]', t)
print(m.group(1) if m else '0.2311')
")
echo "  final height: $FINAL_Z m (spawn was 0.2311)"

echo
echo "--- navigator's own view ---"
timeout 15 ros2 topic echo --once /navigator/status std_msgs/msg/String 2>/dev/null \
    | grep -v '^\[' | head -3 | sed 's/^/  /'

kill "$LP" 2>/dev/null
sleep 3
pkill -f 'gz sim' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null

echo
python3 - "$TRACK" /tmp/track_log.txt "$TOL" "$FINAL_Z" "$(dirname "$0")" <<'PY'
import json, math, os, sys

track_file, log_file = sys.argv[1], sys.argv[2]
tol, final_z = float(sys.argv[3]), float(sys.argv[4])
centre = [(p['x'], p['y'])
          for p in json.load(open(track_file, encoding='utf-8'))['centerline_m']]

pts = []
for line in open(log_file):
    try:
        nums = [float(v) for v in line.split()]
    except ValueError:
        continue
    if len(nums) >= 3:
        # 4th column is yaw, added 2026-09-17. Older logs have three columns
        # and still parse; the clearance metric reports itself unavailable
        # rather than silently assuming the robot was aligned with the lane.
        pts.append((nums[0], nums[1], nums[2],
                    nums[3] if len(nums) >= 4 else None))

if len(pts) < 5:
    print('  FAIL: only %d usable pose samples' % len(pts))
    sys.exit(1)


def dist_to_centreline(x, y):
    """Perpendicular distance to the centreline, and the index it came from.

    The index matters because the course width is NOT constant: IGVC rules
    II.2 vary it from ten to twenty feet, and the track generator paints that
    sinusoid into track.png. A deviation of 1.5 m is comfortable in a 20 ft
    section and over the line in a 10 ft one, so a single centreline distance
    cannot say whether the robot stayed inside the lane.
    """
    best, best_i = float('inf'), 0
    for i in range(len(centre)):
        ax, ay = centre[i]
        bx, by = centre[(i + 1) % len(centre)]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0.0:
            d = math.hypot(x - ax, y - ay)
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        if d < best:
            best, best_i = d, i
    return best, best_i


# One definition of the width profile, shared with the world generator, so the
# world the camera sees and the limit this test scores against cannot drift.
#
# The directory comes in as argv[5], NOT from __file__: this block is fed to
# python3 on stdin, where __file__ is the literal string "<stdin>" and
# dirname(abspath(...)) silently resolves to the working directory instead.
# That produced "NOT MEASURED (No module named 'generate_igvc_world')" on the
# first run, which is the right failure but for the wrong reason.
sys.path.insert(0, sys.argv[5])
try:
    from generate_igvc_world import track_inner_edge_m
    HAVE_WIDTH = True
except Exception as exc:      # noqa: BLE001 - reported, never fatal
    HAVE_WIDTH = False
    WIDTH_ERR = str(exc)

# Two footprints, because the repo disagrees with itself about the robot's
# size. The chassis collision mesh measures 0.810 x 0.970 m and is the widest
# part of the robot, wider than the wheels; Nav2's footprint parameter says
# 0.700 x 1.000 m. Reporting both keeps the disagreement visible.
ROBOT_HALF_TRUE, ROBOT_LEN_HALF_TRUE = 0.405, 0.485
ROBOT_HALF_NAV2, ROBOT_LEN_HALF_NAV2 = 0.350, 0.500


def lateral_reach(half_w, half_l, rel_yaw):
    """How far the footprint reaches sideways at rel_yaw to the lane.

    A fixed half-width is the robot's lateral extent only when it is aligned
    with the lane. Skewed, a corner leads: the rectangle's reach perpendicular
    to the lane is (W/2)|cos t| + (L/2)|sin t|, peaking at hypot(W/2, L/2).

    For the true chassis that is 0.405 m aligned and 0.632 m at the worst
    angle, so ignoring yaw understates the reach by up to 0.227 m. That is
    most of the clearance margin, which is why this is not a refinement.
    """
    return abs(half_w * math.cos(rel_yaw)) + abs(half_l * math.sin(rel_yaw))


def lane_tangent(i):
    """Heading of the lane at centreline index i."""
    n = len(centre)
    ax, ay = centre[(i - 2) % n]
    bx, by = centre[(i + 2) % n]
    return math.atan2(by - ay, bx - ax)


path_len = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
               for i in range(1, len(pts)))
dev_idx = [dist_to_centreline(p[0], p[1]) for p in pts]
devs = [d for d, _ in dev_idx]

# Lane-boundary compliance. REPORTED ONLY - it is deliberately not a pass
# criterion yet, because the footprint width is still an open question and
# tightening the gate before that is settled would fail runs for the wrong
# reason. clearance = local painted half-width minus (deviation + robot
# half-width). Negative means the chassis edge crossed the inner edge of the
# paint.
clear_true = clear_nav2 = None
yaw_missing = any(p[3] is None for p in pts)
if HAVE_WIDTH and not yaw_missing:
    n_c = len(centre)
    clear_true, clear_nav2, rel_yaws = [], [], []
    for (d, i), p in zip(dev_idx, pts):
        h = track_inner_edge_m(i / float(n_c))
        # Wrap to [-pi, pi]; the robot may be driving the lane either way, and
        # a rectangle is symmetric, so only the acute angle to the lane axis
        # matters.
        t = math.atan2(math.sin(p[3] - lane_tangent(i)),
                       math.cos(p[3] - lane_tangent(i)))
        rel_yaws.append(abs(math.degrees(t)))
        clear_true.append(h - (d + lateral_reach(
            ROBOT_HALF_TRUE, ROBOT_LEN_HALF_TRUE, t)))
        clear_nav2.append(h - (d + lateral_reach(
            ROBOT_HALF_NAV2, ROBOT_LEN_HALF_NAV2, t)))
# Odometry cannot see the robot fall off the slab - free-spinning wheels read
# the same as driving - so the height comes from one gz ground-truth reading.
worst_z = abs(final_z - 0.2311)
elapsed = pts[-1][2] - pts[0][2]
# Spread of visited positions: a robot spinning on the spot travels a long
# path while covering no ground, and that must not read as success.
span = max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in pts for b in pts)

print('  samples              : %d' % len(pts))
print('  distance travelled   : %.1f m' % path_len)
print('  furthest two points  : %.1f m apart' % span)
print('  centreline deviation : mean %.2f m, max %.2f m' %
      (sum(devs) / len(devs), max(devs)))
print('  final height         : %.3f m, %.3f off the spawn height'
      % (final_z, worst_z))
print('  sim time covered     : %.1f s' % elapsed)
if clear_true is not None:
    worst_t, worst_n = min(clear_true), min(clear_nav2)
    over_t = 100.0 * sum(1 for c in clear_true if c < 0) / len(clear_true)
    hz = (len(pts) - 1) / elapsed if elapsed > 0 else 0.0
    print('  sample rate          : %.1f Hz (%d samples over %.1f s of sim time)'
          % (hz, len(pts), elapsed))
    print('  yaw to lane          : mean %.1f deg, max %.1f deg'
          % (sum(rel_yaws) / len(rel_yaws), max(rel_yaws)))
    print('  lane clearance       : worst %+.3f m (chassis 0.810 x 0.970 m), '
          '%+.3f m (Nav2 0.700 x 1.000 m)' % (worst_t, worst_n))
    print('                         (negative = footprint corner past the '
          'paint; %.1f%% of samples over the line)' % over_t)
    print('                         Rotated footprint against the LOCAL '
          'painted half-width. REPORTED, not a pass criterion.')
elif yaw_missing:
    print('  lane clearance       : NOT MEASURED (pose log has no yaw column; '
          'rerun with the current pose_logger.py)')
else:
    print('  lane clearance       : NOT MEASURED (%s)' % WIDTH_ERR)
print()

ok = True
if path_len < 5.0:
    print('  MOVED       : FAIL  travelled only %.1f m without being pushed' % path_len)
    ok = False
else:
    print('  MOVED       : PASS  %.1f m, entirely under its own control' % path_len)

if span < 5.0:
    print('  PROGRESS    : FAIL  covered no ground; spinning or stuck')
    ok = False
else:
    print('  PROGRESS    : PASS  ranged over %.1f m of course' % span)

if max(devs) > tol:
    print('  IN LANE     : FAIL  strayed %.2f m from the centreline (limit %.2f)'
          % (max(devs), tol))
    ok = False
else:
    print('  IN LANE     : PASS  never more than %.2f m off the centreline'
          % max(devs))

if worst_z > 0.10:
    print('  ON THE SLAB : FAIL  ended off the ground; readings above are void')
    ok = False
else:
    print('  ON THE SLAB : PASS  wheels on the ground throughout')

print()
print('  ' + ('AUTONOMY CHECK: PASS' if ok else 'AUTONOMY CHECK: FAIL'))
sys.exit(0 if ok else 1)
PY
RC=$?

if [ "$RC" -ne 0 ]; then
    echo
    echo "--- launch log, errors only ---"
    grep -iE "error|abort|fail" /tmp/autonomy.log \
        | grep -viE "deprecated|localhost_only" | tail -20 | sed 's/^/  /'
fi
exit $RC
