#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Does the robot drive the IGVC course BY ITSELF, and does it stay inside the
# painted lane?
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
#   * its chassis crosses the painted boundary,  -> LANE, see below
#     or that cannot be measured
#   * it leaves the ground slab                  -> drove off the world
#   * it moves but covers no NEW course          -> spinning or stuck in place
#   * it touches a barrel, or touches cannot be  -> BARRELS, see below
#     measured
#
# Docker: run INSIDE the Gazebo container. pixi: `pixi run autonomy`.
#
# Nothing here depends on how fast the machine is. The watch window is
# SIMULATION time and the start is a readiness check, so a slow machine (tier
# C, or a Mac) runs the same test for longer in wall-clock terms rather than a
# shorter, easier one. Until 2026-09-24 both were wall-clock: the same code
# drove 83.0 m on one day and 47.4 m on another, because the simulator ran at
# about half real time on the second.
#
#   DURATION=100   how long to watch, in SIMULATION seconds (default 100)
#   STARTUP_LIMIT=300  wall-clock ceiling on waiting for the robot to start
#                  driving itself. A ceiling, not a wait.
#   NAV2_DELAY=25  delay Nav2's own bringup, if it loses the race with
#                  Gazebo on a loaded machine
#   BRINGUP_RETRIES=1  how many times a run may start over after Nav2's
#                  lifecycle manager aborts its bring-up ("Failed to bring up
#                  all requested nodes. Aborting bringup."), default 1. That abort
#                  is a start-up fault (2 of 59 saved bring-ups, 2026-10-01 and
#                  2026-10-09, always at collision_monitor's get_state, which the
#                  manager gives 2 s and never retries): after it nothing can
#                  navigate, so waiting out the ceiling only wastes 5 to 9 minutes.
#                  The attempt is torn down, loudly reported and counted, and the
#                  run starts again from a fresh simulator. A run that needed a
#                  retry prints "NAV2 BRINGUP RETRIES: n" with n > 0 as its last
#                  line, and a gate that wants "clean" passes must grep for 0.
#                  BRINGUP_RETRIES=0 makes the first abort a FAIL at once (exit 1,
#                  "NOT GRADED"). Aborts are never silent and never become a PASS
#                  of anything they did not measure: the graded attempt is always
#                  a complete run of its own.
#   LAUNCH_ARGS="a:=b c:=d"  extra arguments for gazebo_nav_test.launch.py,
#                  appended last, for running the same check on another
#                  configuration (a perception source, say). The default run
#                  passes none, so the default meaning of this check is
#                  unchanged: ground-truth lanes and barrels.
#   ANALYSE_ONLY=<track_log>  do not run a simulator: score a saved run. The
#                  same Python block that grades a live run reads the file.
#                  Optional companions, because a live run measures them and a
#                  file cannot: CONTACT_LOG=<file> (default: none, so BARRELS
#                  reads NOT MEASURED), CONTACT_SENSORS=<n> (default 0), and
#                  FINAL_Z=<metres> (default nan, so ON THE SLAB reads NOT
#                  MEASURED). Unmeasured stays a FAIL; nothing is assumed.
#
# LANE (replaced IN LANE on 2026-10-07): the chassis footprint must never cross
# the painted boundary, and a run in which that cannot be measured fails. IN
# LANE failed a run whose distance from the centreline passed a flat 2.00 m.
# The rules do not judge distance from the centre. They judge the line: 2026
# rules p.11, "crossing internal lines ... will be judged an E-Stop end of run
# with penalty"; p.12 measures the distance to where the vehicle crossed the
# boundary edge. (A partial crossing with part of the vehicle still in bounds
# is a 5 ft "Careless Driving" ticket on p.12, so LANE is stricter than that
# ticket and follows p.11.) The lane is 10 to 20 ft wide, so the half-width
# runs from 1.52 to 3.05 m, and 2.00 m was too loose where it is narrow and too
# strict where it is wide: runs with 0.0% of samples over the paint and a
# positive clearance failed at 2.04 m (2026-09-17) and 2.07 m (2026-10-01).
# The centreline deviation is still printed, as a number.
#
# Resolution: the painted half-width is looked up by nearest centreline index,
# which steps by up to 2.8 cm, so a clearance is good to about 3 cm. Margins
# smaller than that are not a measurement (docs/GAZEBO_SETUP.md 10.9).
#
# BARRELS (added 2026-10-01): touching any barrel is a FAIL, and so is being
# unable to tell. Two reasons. The IGVC rules penalise any touch. And a touch
# voids the other readings: they come from odometry, which keeps counting
# while the wheels slip against a static barrel. Measured: a robot pinned
# against barrel 4 from 39 s to the end read as a normal 89.4 m lap, MOVED,
# PROGRESS and IN LANE all PASS, while ground truth put it 14.9 m from where
# odometry said. Ground-truth runs touch no barrel (measured the same day), so
# a correct run loses nothing to this criterion.
# ---------------------------------------------------------------------------
set -o pipefail
. "$(dirname "$0")/igvc_env.sh" || exit 1

DURATION="${DURATION:-100}"
STARTUP_LIMIT="${STARTUP_LIMIT:-300}"
# Passed through to the launch file, which delays Nav2 to stop it racing
# Gazebo. Its own default is 15.0 and neither check script could reach it
# before, so the documented fix for a lost race was unreachable from here.
NAV2_DELAY="${NAV2_DELAY:-}"
BRINGUP_RETRIES="${BRINGUP_RETRIES:-1}"
BRINGUP_ABORTS=0
case "$BRINGUP_RETRIES" in
    ''|*[!0-9]*) echo "FAIL: BRINGUP_RETRIES=$BRINGUP_RETRIES is not a whole number."; exit 1 ;;
esac
TRACK="$REPO/IGVC_track_generator/track_points.json"

# TOL was the old IN LANE limit. Nothing reads it now; say so rather than
# letting a stale habit look as though it still does something.
[ -n "${TOL:-}" ] && echo "note: TOL is no longer read; LANE fails on a negative clearance, not a distance"

if [ -n "${ANALYSE_ONLY:-}" ]; then
    # Scoring a saved run. Nothing is built, started or measured, so there
    # is no preflight either. What a live run measures and a file cannot
    # carry comes in from the environment, and defaults to "not measured",
    # which fails.
    if [ ! -f "$ANALYSE_ONLY" ]; then
        echo "FAIL: ANALYSE_ONLY=$ANALYSE_ONLY is not a file."
        exit 1
    fi
    TRACK_LOG="$ANALYSE_ONLY"
    CONTACT_LOG="${CONTACT_LOG:-}"
    CONTACT_SENSORS="${CONTACT_SENSORS:-0}"
    FINAL_Z="${FINAL_Z:-nan}"
    echo
    echo "=============================================================="
    echo " Gazebo autonomy check: ANALYSE ONLY, no simulator was run"
    echo "=============================================================="
    echo "track log    : $TRACK_LOG"
    echo "contact log  : ${CONTACT_LOG:-none} (${CONTACT_SENSORS} sensor(s) declared)"
    echo "final height : $FINAL_Z"
    echo
else
    TRACK_LOG=/tmp/track_log.txt
    CONTACT_LOG=/tmp/contact_log.txt

    # Refuse to run on top of an existing simulator. A run with two of them looks
    # plausible and means nothing - see sim_preflight.sh.
    . "$(dirname "$0")/sim_preflight.sh"
    FORCE=1 sim_preflight || exit 1

    echo "--- building ---"
    igvc_build || exit 1
    echo "    ok"

    echo
    echo "=============================================================="
    echo " Gazebo autonomy check: does it drive the course itself?"
    echo "=============================================================="
    echo "watching for ${DURATION} s of SIMULATION time"
    echo "this script never publishes /cmd_vel"
    echo

    ARGS="headless:=true"
    [ "${RVIZ:-0}" = "1" ] && ARGS="headless:=false rviz:=true"
    [ -n "$NAV2_DELAY" ] && ARGS="$ARGS nav2_delay:=$NAV2_DELAY"
    [ -n "${LAUNCH_ARGS:-}" ] && ARGS="$ARGS $LAUNCH_ARGS" \
        && echo "extra launch arguments: $LAUNCH_ARGS"

    # One attempt is: launch, then wait for the robot to drive. If Nav2's
    # lifecycle manager aborts its bring-up (see BRINGUP_RETRIES at the top) the
    # attempt is not a result, so it is torn down, reported, counted, and run
    # again up to BRINGUP_RETRIES times. Every abort is printed here and the
    # count is printed again on the last line of the run. Nothing else is retried.
    BRINGUP_ABORTS=0
    ATTEMPT=0
    while :; do
        ATTEMPT=$((ATTEMPT + 1))
        ros2 launch igvc_test_bringup gazebo_nav_test.launch.py $ARGS \
            > /tmp/autonomy.log 2>&1 &
        LP=$!

        # Every barrel touch from launch to the end of the watch window, so a
        # touch during startup counts too. See contact_logger.py for why silence
        # alone proves nothing and what it records instead.
        python3 "$(dirname "$0")/contact_logger.py" "$CONTACT_LOG" \
            > /tmp/contact_logger.out 2>&1 &
        CL=$!

        # Open the watch window when the robot is DRIVING ITSELF, not after a guess.
        #
        # This was `sleep 50` until 2026-09-24: Gazebo, ten Nav2 lifecycle nodes in
        # sequence, then the navigator's first plan. How long that takes is a
        # property of the machine. The thing the window is meant to watch starts
        # when the robot moves, so wait for exactly that. If it never moves, the
        # ceiling expires, the window runs anyway, and MOVED fails loudly, which is
        # the right outcome.
        #
        # --abort-log: Nav2's lifecycle manager gives up for good on the first
        # failed transition, so a log line saying so ends the wait at once (exit 3)
        # instead of after the ceiling. Before 2026-10-09 such a run spent the whole
        # ceiling and the whole watch window (566 s) to report 0.0 m.
        echo "waiting for the robot to start driving itself (ceiling ${STARTUP_LIMIT}s)..."
        python3 "$(dirname "$0")/wait_ready.py" moving "$STARTUP_LIMIT" 0.3 \
            --abort-log /tmp/autonomy.log
        WRC=$?
        [ "$WRC" -ne 3 ] && break

        BRINGUP_ABORTS=$((BRINGUP_ABORTS + 1))
        cp /tmp/autonomy.log "/tmp/autonomy.bringup_abort${BRINGUP_ABORTS}.log" 2>/dev/null
        echo
        echo "!! NAV2 BRING-UP ABORT #${BRINGUP_ABORTS} (attempt ${ATTEMPT}): the lifecycle manager gave up."
        echo "!! This is a start-up fault, not a navigation result. Launch log kept as"
        echo "!! /tmp/autonomy.bringup_abort${BRINGUP_ABORTS}.log"
        kill -INT "$CL" 2>/dev/null
        kill "$LP" 2>/dev/null
        sleep 3
        sim_teardown
        if [ "$BRINGUP_ABORTS" -gt "$BRINGUP_RETRIES" ]; then
            echo "FAIL: Nav2 bring-up aborted ${BRINGUP_ABORTS} time(s) and BRINGUP_RETRIES=${BRINGUP_RETRIES};"
            echo "      the robot was never given a chance to drive, so the run is not graded."
            echo
            echo "  AUTONOMY CHECK: FAIL (Nav2 bring-up aborted; the robot was not graded)"
            echo "NAV2 BRINGUP RETRIES: ${BRINGUP_ABORTS}"
            exit 1
        fi
        echo "!! Starting a fresh simulator (retry ${BRINGUP_ABORTS} of ${BRINGUP_RETRIES})."
        echo
    done

    if ! kill -0 "$LP" 2>/dev/null; then
        kill -INT "$CL" 2>/dev/null   # sim_teardown does not match a script
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
    # How many barrel contact sensors are live, counted as Gazebo PUBLISHERS on
    # /sim/barrel_contacts (one per barrel). A world generated before 2026-10-01
    # has none, and its silence would read as "never touched a barrel". Not
    # `gz topic -l`: that lists the topic in either case, because the bridge
    # SUBSCRIBES to it (measured with both worlds, 2026-10-01).
    CONTACT_SENSORS=$(timeout 30 gz topic -i -t /sim/barrel_contacts 2>/dev/null \
        | awk '/^Publishers/ {p = 1; next} /^Subscribers/ {p = 0}
               p && /gz.msgs.Contacts/ {n++} END {print n + 0}')

    echo "watching for ${DURATION} s of simulation time..."
    python3 "$(dirname "$0")/pose_logger.py" "$TRACK_LOG" "$DURATION" $SPAWN "${POSE_HZ:-10.0}" sim
    kill -INT "$CL" 2>/dev/null
    wait "$CL" 2>/dev/null
    cat /tmp/contact_logger.out

    # One ground-truth reading, for the one thing odometry cannot tell us: whether
    # the robot is still on the ground slab. Wheels spinning in mid-air produce
    # perfectly healthy odometry.
    #
    # If the read fails, say so. It used to fall back to the spawn height, which
    # made ON THE SLAB pass without anything having been measured, and on a slow
    # machine this gz-transport call is exactly the one that times out.
    timeout 60 gz model -m igvc_robot -p 2>/dev/null > /tmp/gzpose.txt
    FINAL_Z=$(python3 -c "
import re
t = open('/tmp/gzpose.txt').read()
m = re.search(r'\[\s*-?[0-9.]+\s+-?[0-9.]+\s+(-?[0-9.]+)\s*\]', t)
print(m.group(1) if m else 'nan')
")
    echo "  final height: $FINAL_Z m (spawn was 0.2311)"

    echo
    echo "--- navigator's own view ---"
    timeout 15 ros2 topic echo --once /navigator/status std_msgs/msg/String 2>/dev/null \
        | grep -v '^\[' | head -3 | sed 's/^/  /'

    kill "$LP" 2>/dev/null
    sleep 3
    # Everything the launch started, not only gz sim and the bridge. See
    # sim_teardown in sim_preflight.sh for what used to be left running.
    sim_teardown
fi

echo
python3 - "$TRACK" "$TRACK_LOG" "$FINAL_Z" "$(dirname "$0")" \
    "$CONTACT_LOG" "$CONTACT_SENSORS" <<'PY'
import json, math, os, sys

track_file, log_file = sys.argv[1], sys.argv[2]
final_z = float(sys.argv[3])
contact_file, contact_sensors = sys.argv[5], int(sys.argv[6] or 0)
n_barrels = len(json.load(open(track_file, encoding='utf-8')).get('obstacles_m', []))
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
# The directory comes in as argv[4], NOT from __file__: this block is fed to
# python3 on stdin, where __file__ is the literal string "<stdin>" and
# dirname(abspath(...)) silently resolves to the working directory instead.
# That produced "NOT MEASURED (No module named 'generate_igvc_world')" on the
# first run, which is the right failure but for the wrong reason.
sys.path.insert(0, sys.argv[4])
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

# Lane-boundary compliance, and the LANE criterion below grades it on the
# chassis footprint (clear_true), the larger of the two and so the stricter.
# clearance = local painted half-width minus the rotated footprint's reach
# sideways from the centreline. Negative means a chassis corner crossed the
# inner edge of the paint. clear_nav2 stays in the report: the repo still
# disagrees with itself about the robot's size.
clear_true = clear_nav2 = None
nonfinite = 0
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
    # A NaN yaw or position gives a NaN clearance, which min() and "< 0" both
    # skip, so one bad sample would otherwise be invisible. Any non-finite
    # sample means the measurement as a whole is missing.
    nonfinite = sum(1 for c in clear_true if not math.isfinite(c))
    if nonfinite:
        clear_true = clear_nav2 = None
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
print('  centreline deviation : mean %.2f m, max %.2f m (reported, not a '
      'criterion)' % (sum(devs) / len(devs), max(devs)))
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
          'painted half-width. The chassis figure is the LANE criterion.')
elif yaw_missing:
    print('  lane clearance       : NOT MEASURED (pose log has no yaw column; '
          'rerun with the current pose_logger.py)')
elif nonfinite:
    print('  lane clearance       : NOT MEASURED (%d sample(s) have a '
          'non-finite pose or yaw)' % nonfinite)
else:
    print('  lane clearance       : NOT MEASURED (%s)' % WIDTH_ERR)

# Barrel contacts, from contact_logger.py. Silence is the normal result AND
# what a broken chain produces, so "none" is only printed when Gazebo had
# contact sensors publishing and the bridge offered the topic in ROS.
import re
touches, publishers_seen = {}, 0
if os.path.exists(contact_file):
    for line in open(contact_file):
        if line.startswith('# publishers_seen'):
            publishers_seen = int(line.split()[-1])
            continue
        f = line.split()
        if len(f) != 4:
            continue
        part = f[2].split('::')[-1]
        part = part.split('lump__', 1)[-1]
        part = re.sub(r'_collision(_\d+)?$', '', part)
        b = touches.setdefault(f[1], {'t': [], 'parts': set()})
        b['t'].append(float(f[0]))
        b['parts'].add(part)
contacts_measured = contact_sensors > 0 and publishers_seen > 0
if not contact_sensors:
    print('  barrel contacts      : NOT MEASURED (no barrel contact sensor in the '
          'world; regenerate it with generate_igvc_world.py)')
elif publishers_seen == 0:
    print('  barrel contacts      : NOT MEASURED (nothing published '
          '/sim/barrel_contacts in ROS; is it in gazebo_bridge.yaml?)')
elif not touches:
    print('  barrel contacts      : none (%d barrel sensor(s) live)' % contact_sensors)
else:
    print('  barrel contacts      : %d barrel(s) touched' % len(touches))
    for name in sorted(touches, key=lambda n: min(touches[n]['t'])):
        ts = touches[name]['t']
        # Messages arrive at up to the physics rate while touching, so count
        # 0.1 s bins of simulation time rather than messages.
        secs = 0.1 * len({int(t * 10.0) for t in ts})
        print('                         %s at %.1f s sim, about %.1f s in '
              'contact, by %s' % (name, min(ts), secs,
                                  ', '.join(sorted(touches[name]['parts']))))
    print('                         Every reading above comes from odometry, '
          'which keeps counting while')
    print('                         the wheels slip against a barrel: treat '
          'them as void after %.1f s.'
          % min(min(b['t']) for b in touches.values()))
if contacts_measured and contact_sensors != n_barrels:
    print('                         WARNING: %d sensor(s) for %d barrel(s) in '
          'the track file' % (contact_sensors, n_barrels))
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

# LANE: no sample may have the chassis footprint past the painted boundary,
# and a run whose clearance could not be computed fails rather than passes
# (no yaw column, no width profile, or a non-finite sample: see above).
# Centreline deviation is a number in the report, not a limit.
if clear_true is None:
    print('  LANE        : FAIL  NOT MEASURED, see lane clearance above')
    ok = False
elif worst_t < 0:
    print('  LANE        : FAIL  chassis crossed the paint: worst clearance '
          '%+.3f m, %.1f%% of samples over the line' % (worst_t, over_t))
    ok = False
else:
    print('  LANE        : PASS  chassis never crossed the paint, worst '
          'clearance %+.3f m' % worst_t)

if worst_z != worst_z:   # NaN: the ground-truth height could not be read
    print('  ON THE SLAB : FAIL  NOT MEASURED, gz model -p returned no pose')
    ok = False
elif worst_z > 0.10:
    print('  ON THE SLAB : FAIL  ended off the ground; readings above are void')
    ok = False
else:
    print('  ON THE SLAB : PASS  wheels on the ground throughout')

if not contacts_measured:
    print('  BARRELS     : FAIL  NOT MEASURED, see barrel contacts above')
    ok = False
elif touches:
    print('  BARRELS     : FAIL  touched %s; readings above are void'
          % ', '.join(sorted(touches)))
    ok = False
else:
    print('  BARRELS     : PASS  touched no barrel')

print()
print('  ' + ('AUTONOMY CHECK: PASS' if ok else 'AUTONOMY CHECK: FAIL'))
sys.exit(0 if ok else 1)
PY
RC=$?

if [ "$RC" -ne 0 ] && [ -z "${ANALYSE_ONLY:-}" ]; then
    echo
    echo "--- launch log, errors only ---"
    grep -iE "error|abort|fail" /tmp/autonomy.log \
        | grep -viE "deprecated|localhost_only" | tail -20 | sed 's/^/  /'
fi

# The last line of every live run, so a gate can count start-up flakes with one
# grep. 0 is a clean run; n > 0 means n earlier attempts never got to drive.
if [ -z "${ANALYSE_ONLY:-}" ]; then
    echo
    if [ "$BRINGUP_ABORTS" -gt 0 ]; then
        echo "note: the result above is attempt $((BRINGUP_ABORTS + 1)); the ${BRINGUP_ABORTS} earlier launch(es) aborted in Nav2 bring-up (logs: /tmp/autonomy.bringup_abort*.log)"
    fi
    echo "NAV2 BRINGUP RETRIES: ${BRINGUP_ABORTS}"
fi
exit $RC
