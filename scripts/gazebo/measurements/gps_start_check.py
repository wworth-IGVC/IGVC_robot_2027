#!/usr/bin/env python3
"""Does the GPS mission start when the robot REACHES the start waypoint, and not before?

Run inside the simulator's container with gazebo_sim.launch.py up and nothing
else driving the robot (gps_start_check.sh does all of that).  Three arms, each
with a fresh mission_planner_node reading the simulator's real /fix and odometry:

  P  the shipped config, whose waypoints are the placeholder (0, 0).
     PASS needs: it never anchors, AND its log names the placeholder.  (Before
     2026-10-08 it never anchored either, but only said "ignoring GPS fix until
     ... prefixes ...", which sends nobody to the right file.)
  F  waypoints entered, the start waypoint 8 m ahead of the robot, the gate 4 m,
     the robot at rest.  PASS needs: it has NOT anchored after 12 s of fixes.
     Before 2026-10-08 the gate compared the first three digits of latitude and
     longitude, which match for anything within about 11 km, so it anchored on
     the first fix.
  N  the same, the robot driven straight at the start waypoint at 0.5 m/s.
     PASS needs: it anchors; it does so within the gate (plus one 1 Hz fix of
     travel) of the start waypoint; and the virtual obstacles it publishes sit
     where ground truth puts them in odom, within 0.5 m, after the robot has
     driven more than 3 m (so an anchor that ignored the odom position would be
     3 m or more off, and fail).

The negative control is the old code itself: put a copy of the old
igvc_lane_detection/ (with the old mission_planner.py) in a directory and set
PLANNER_PYTHONPATH to it; arms P and F must FAIL on it, and do.  The datum is a
PLACEHOLDER (generate_igvc_world.py), which is fine: only differences matter.

Odom in the simulator is world-aligned with its origin at the spawn point
(gazebo_odom_shim), so truth in odom is world minus spawn.  That is exactly why
the rotation of real-robot odom needs its own test (Task 5c).
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from generate_igvc_world import DATUM_LAT_DEG, DATUM_LON_DEG  # noqa: E402

REPO = os.environ.get('IGVC_WORKSPACE_ROOT',
                      os.path.abspath(os.path.join(
                          os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')))
TRACK = os.path.join(REPO, 'IGVC_track_generator', 'track_points.json')
ODOM = '/front_zed_camera_x/zed_node/odom'
RADIUS_M = 4.0
START_AHEAD_M = 8.0
TOL_M = 0.5
OUT = os.environ.get('GPS_CHECK_OUT', '/tmp/gps_start_check')

A = 6378137.0
E2 = 6.69437999014e-3
phi = math.radians(DATUM_LAT_DEG)
M = A * (1 - E2) / (1 - E2 * math.sin(phi) ** 2) ** 1.5
N = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)


def world_to_latlon(x, y):
    return (DATUM_LAT_DEG + math.degrees(y / M),
            DATUM_LON_DEG + math.degrees(x / (N * math.cos(phi))))


track = json.load(open(TRACK, encoding='utf-8'))
sp = track['robot_start_pose']
# gps_start_check.sh spawns the robot in open ground at the course centre,
# facing east, so the 8 m drive and the waypoints beyond it stay on the slab
# and clear of barrels; it passes that pose here.  The course's own start faces
# the slab's edge.
SPAWN = (float(os.environ.get('SPAWN_X', sp['position_m']['x'])),
         float(os.environ.get('SPAWN_Y', sp['position_m']['y'])))
YAW = float(os.environ.get('SPAWN_YAW', sp['yaw_rad']))
fwd = (math.cos(YAW), math.sin(YAW))
left = (-math.sin(YAW), math.cos(YAW))

# world points: the start waypoint straight ahead, the obstacles beside the
# line further on, the end beyond.  All must lie on the ground slab, which is
# the track's bounding box plus 6 m (generate_igvc_world.py).
S = (SPAWN[0] + START_AHEAD_M * fwd[0], SPAWN[1] + START_AHEAD_M * fwd[1])
O1 = (S[0] + 3 * fwd[0] + 2 * left[0], S[1] + 3 * fwd[1] + 2 * left[1])
O2 = (S[0] + 6 * fwd[0] - 2 * left[0], S[1] + 6 * fwd[1] - 2 * left[1])
E = (S[0] + 9 * fwd[0], S[1] + 9 * fwd[1])
xs = [p['x'] for p in track['centerline_m']]
ys = [p['y'] for p in track['centerline_m']]
for name, pt in (('S', S), ('O1', O1), ('O2', O2), ('E', E)):
    if not (min(xs) - 5.5 < pt[0] < max(xs) + 5.5 and min(ys) - 5.5 < pt[1] < max(ys) + 5.5):
        print('  FAIL (setup): waypoint %s %s is off the ground slab' % (name, pt))
        sys.exit(2)

os.makedirs(OUT, exist_ok=True)
overlay = {'mission_planner_node': {'ros__parameters': {
    'use_sim_time': True,
    'auto_start': False,
    'ramp_enabled': False,
    'gps_start_match_radius_m': RADIUS_M,
    'waypoint_lats': [world_to_latlon(*p)[0] for p in (S, O1, O2, E)],
    'waypoint_lons': [world_to_latlon(*p)[1] for p in (S, O1, O2, E)],
    'waypoint_kinds': ['start', 'obstacle', 'obstacle', 'end'],
}}}
overlay_path = os.path.join(OUT, 'entered_waypoints.yaml')
yaml.safe_dump(overlay, open(overlay_path, 'w', encoding='utf-8'))
shipped = os.path.join(get_package_share_directory('igvc_test_bringup'),
                       'config', 'mission_planner_config.yaml')

rclpy.init()
node = Node('gps_start_check')
state = {'poses': None, 'odom': None, 'anchored_at': None}
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def on_obstacles(m):
    if len(m.poses) and state['anchored_at'] is None:
        state['anchored_at'] = (time.time(), state['odom'])
    state['poses'] = [(p.position.x, p.position.y) for p in m.poses]


def on_odom(m):
    state['odom'] = (m.pose.pose.position.x, m.pose.pose.position.y)


node.create_subscription(PoseArray, '/mission/virtual_obstacles', on_obstacles, latched)
node.create_subscription(Odometry, ODOM, on_odom, qos_profile_sensor_data)
cmd = node.create_publisher(Twist, '/cmd_vel', 10)


def spin(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)


def drive(vx):
    t = Twist()
    t.linear.x = vx
    cmd.publish(t)


def start_planner(arm, files):
    state['poses'], state['anchored_at'] = None, None
    log = open(os.path.join(OUT, 'planner_%s.log' % arm), 'w')
    args = ['ros2', 'run', 'igvc_lane_detection', 'mission_planner_node',
            '--ros-args']
    for f in files:
        args += ['--params-file', f]
    # Its own process group: `ros2 run` starts the node as a child, and
    # terminating only `ros2 run` leaves the node running into the next arm
    # (seen on 2026-10-08: arm F's planner was still publishing during arm N).
    env = dict(os.environ)
    if os.environ.get('PLANNER_PYTHONPATH'):
        # The negative control: a directory holding an older
        # igvc_lane_detection/mission_planner.py goes first on the planner's
        # path, so the same test runs against the old code.
        env['PYTHONPATH'] = (os.environ['PLANNER_PYTHONPATH'] + os.pathsep
                             + env.get('PYTHONPATH', ''))
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=env)
    return proc, log


def stop_planner(proc, log):
    for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=wait)
            break
        except subprocess.TimeoutExpired:
            continue
    # and nothing of it may be left publishing
    left = subprocess.run(['pgrep', '-f', 'mission_planner_node'],
                          capture_output=True, text=True).stdout.split()
    if left:
        print('  FAIL (setup): a mission_planner_node outlived its arm: %s' % left)
        sys.exit(2)
    log.close()


def planner_text(arm):
    return open(os.path.join(OUT, 'planner_%s.log' % arm), encoding='utf-8',
                errors='replace').read()


def wait_odom(limit=30.0):
    t0 = time.time()
    while state['odom'] is None and time.time() - t0 < limit:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state['odom'] is None:
        print('  FAIL (setup): no odometry on %s' % ODOM)
        sys.exit(2)


def world_of_odom(o):
    return (SPAWN[0] + o[0], SPAWN[1] + o[1])


wait_odom()
results = {}

# ---- arm P: the shipped placeholder config
proc, log = start_planner('P', [shipped])
spin(15.0)
stop_planner(proc, log)
text = planner_text('P')
never = state['poses'] in (None, []) and state['anchored_at'] is None
says = 'placeholder' in text.lower()
results['P'] = never and says
print('  arm P  shipped placeholder config : anchored=%s, log names the placeholder=%s'
      % (not never, says))
print('         PASS' if results['P'] else
      '         FAIL (must never anchor, and must say why)')

# ---- arm F: entered waypoints, robot 8 m away, at rest
proc, log = start_planner('F', [shipped, overlay_path])
spin(12.0)
stop_planner(proc, log)
anchored = state['anchored_at'] is not None
results['F'] = not anchored
print('  arm F  robot %.0f m from the start waypoint, gate %.0f m, at rest : anchored=%s'
      % (START_AHEAD_M, RADIUS_M, anchored))
print('         PASS' if results['F'] else
      '         FAIL (it anchored before the robot reached the start waypoint)')

# ---- arm N: drive at the start waypoint
proc, log = start_planner('N', [shipped, overlay_path])
spin(4.0)                                   # let it subscribe and see a fix
t0 = time.time()
while state['anchored_at'] is None and time.time() - t0 < 40.0:
    drive(0.5)
    spin(0.1)
drive(0.0)
spin(1.0)
stop_planner(proc, log)
if state['anchored_at'] is None:
    results['N'] = False
    print('  arm N  drove toward the start waypoint: never anchored')
    print('         FAIL')
else:
    t_anchor, odom_at = state['anchored_at']
    here = world_of_odom(odom_at) if odom_at else None
    dist = math.hypot(here[0] - S[0], here[1] - S[1]) if here else float('nan')
    driven = math.hypot(odom_at[0], odom_at[1]) if odom_at else float('nan')
    poses = state['poses'] or []
    truth = [(O1[0] - SPAWN[0], O1[1] - SPAWN[1]), (O2[0] - SPAWN[0], O2[1] - SPAWN[1])]
    errs = []
    if len(poses) == 2:
        errs = [math.hypot(p[0] - t[0], p[1] - t[1]) for p, t in zip(poses, truth)]
    # one 1 Hz fix is 0.5 m of travel, so anchoring may lag the gate by that
    near = dist <= RADIUS_M + 1.0
    placed = len(errs) == 2 and max(errs) <= TOL_M
    moved = driven > 3.0
    results['N'] = near and placed and moved
    print('  arm N  anchored when the robot was %.1f m from the start waypoint '
          '(gate %.0f m), after driving %.1f m' % (dist, RADIUS_M, driven))
    print('         virtual obstacles vs ground truth in odom: %s'
          % (', '.join('%.2f m' % e for e in errs) or 'wrong count %d' % len(poses)))
    print('         an anchor that ignored the odom position would be %.1f m off'
          % driven)
    print('         PASS' if results['N'] else
          '         FAIL (near=%s placed=%s moved=%s)' % (near, placed, moved))

ok = all(results.values())
print()
print('  VERDICT : %s   (P %s, F %s, N %s)' % (
    'PASS' if ok else 'FAIL', *['pass' if results.get(k) else 'FAIL' for k in 'PFN']))
sys.exit(0 if ok else 1)
