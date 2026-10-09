#!/usr/bin/env python3
"""Do GPS waypoints land in the right place when odometry's axes follow the
robot's start heading, as on the real robot?

One start heading per run (SPAWN_YAW, radians); gps_heading_check.sh runs it at
several, each with a fresh simulator started with odom_axes:=robot, so odom x
points wherever the robot faced at spawn (gazebo_odom_shim as a pass-through).
The robot spawns at the course centre, drives straight at 0.5 m/s at a start
waypoint 14 m ahead (gate 3 m, so at least 11 m of driving comes before the
anchor), and the check compares the two virtual obstacles the planner publishes
with ground truth expressed in that odom frame.

PASS: both obstacles within 0.5 m of truth.  The 0.22 m between the GPS
antenna and base_link, which the planner does not correct, is inside that.

Prediction for the planner before 2026-10-08's heading fit: PASS only when the
robot starts facing east (yaw 0), and FAIL by the rotation at every other yaw,
because it treats odom x and y as east and north.  PLANNER_PARAMS (extra
key=value pairs, space separated) is passed to the planner, so
PLANNER_PARAMS="odom_yaw_fit_enabled:=false" is the negative control on new
code; PLANNER_PYTHONPATH runs older code, as in gps_start_check.py.
"""
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

ODOM = '/front_zed_camera_x/zed_node/odom'
START_AHEAD_M = 14.0
RADIUS_M = 3.0
TOL_M = 0.5
OUT = os.environ.get('GPS_CHECK_OUT', '/tmp/gps_heading_check')
SPAWN = (float(os.environ.get('SPAWN_X', '0')), float(os.environ.get('SPAWN_Y', '0')))
YAW = float(os.environ.get('SPAWN_YAW', '0'))
EXTRA = os.environ.get('PLANNER_PARAMS', '').split()

A = 6378137.0
E2 = 6.69437999014e-3
phi = math.radians(DATUM_LAT_DEG)
M = A * (1 - E2) / (1 - E2 * math.sin(phi) ** 2) ** 1.5
N = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)


def world_to_latlon(x, y):
    return (DATUM_LAT_DEG + math.degrees(y / M),
            DATUM_LON_DEG + math.degrees(x / (N * math.cos(phi))))


fwd = (math.cos(YAW), math.sin(YAW))
left = (-math.sin(YAW), math.cos(YAW))
S = (SPAWN[0] + START_AHEAD_M * fwd[0], SPAWN[1] + START_AHEAD_M * fwd[1])
O1 = (S[0] + 3 * fwd[0] + 2 * left[0], S[1] + 3 * fwd[1] + 2 * left[1])
O2 = (S[0] + 6 * fwd[0] - 2 * left[0], S[1] + 6 * fwd[1] - 2 * left[1])
E = (S[0] + 9 * fwd[0], S[1] + 9 * fwd[1])


def to_odom(p):
    """World point to the robot-axes odom frame: origin at the spawn, x along
    the spawn heading."""
    dx, dy = p[0] - SPAWN[0], p[1] - SPAWN[1]
    return (math.cos(YAW) * dx + math.sin(YAW) * dy,
            -math.sin(YAW) * dx + math.cos(YAW) * dy)


os.makedirs(OUT, exist_ok=True)
overlay = {'mission_planner_node': {'ros__parameters': {
    'use_sim_time': True, 'auto_start': False, 'ramp_enabled': False,
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
node = Node('gps_heading_check')
state = {'poses': None, 'odom': None, 'anchored': None}
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def on_obstacles(m):
    if len(m.poses) and state['anchored'] is None:
        state['anchored'] = state['odom']
    state['poses'] = [(p.position.x, p.position.y) for p in m.poses]


node.create_subscription(PoseArray, '/mission/virtual_obstacles', on_obstacles, latched)
node.create_subscription(
    Odometry, ODOM,
    lambda m: state.__setitem__('odom', (m.pose.pose.position.x, m.pose.pose.position.y)),
    qos_profile_sensor_data)
cmd = node.create_publisher(Twist, '/cmd_vel', 10)


def spin(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)


def drive(vx):
    t = Twist()
    t.linear.x = vx
    cmd.publish(t)


env = dict(os.environ)
if os.environ.get('PLANNER_PYTHONPATH'):
    env['PYTHONPATH'] = os.environ['PLANNER_PYTHONPATH'] + os.pathsep + env.get('PYTHONPATH', '')
args = ['ros2', 'run', 'igvc_lane_detection', 'mission_planner_node', '--ros-args',
        '--params-file', shipped, '--params-file', overlay_path]
for kv in EXTRA:
    args += ['-p', kv]
log = open(os.path.join(OUT, 'planner.log'), 'w')
proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True, env=env)

t0 = time.time()
while state['odom'] is None and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.1)
spin(4.0)
t0 = time.time()
while state['anchored'] is None and time.time() - t0 < 50.0:
    drive(0.5)
    spin(0.1)
drive(0.0)
spin(1.5)
for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
    try:
        os.killpg(proc.pid, sig)
        proc.wait(timeout=wait)
        break
    except ProcessLookupError:
        break
    except subprocess.TimeoutExpired:
        continue
log.close()

print('  start heading %.0f deg, odom axes along it (odom_axes:=robot)%s'
      % (math.degrees(YAW), (', planner params: ' + ' '.join(EXTRA)) if EXTRA else ''))
if state['anchored'] is None:
    print('  never anchored')
    print('  VERDICT : FAIL')
    sys.exit(1)
driven = math.hypot(*state['anchored'])
poses = state['poses'] or []
truth = [to_odom(O1), to_odom(O2)]
errs = [math.hypot(p[0] - t[0], p[1] - t[1]) for p, t in zip(poses, truth)] \
    if len(poses) == 2 else []
print('  anchored after %.1f m of driving' % driven)
print('  virtual obstacles in odom: %s' % ', '.join('(%.2f, %.2f)' % p for p in poses))
print('  ground truth in odom     : %s' % ', '.join('(%.2f, %.2f)' % t for t in truth))
print('  errors                   : %s' % (', '.join('%.2f m' % e for e in errs) or 'wrong count'))
ok = len(errs) == 2 and max(errs) <= TOL_M
print('  VERDICT : %s' % ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
