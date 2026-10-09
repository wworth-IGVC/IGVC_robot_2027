#!/usr/bin/env python3
"""Does the GPS mission start BY ITSELF when the lane-following robot reaches
the start waypoint, with nobody calling ~/start_mission?

Run inside the simulator's container with gazebo_nav_test.launch.py up (Nav2
and the lane navigator driving the robot on ground truth); gps_mission_check.sh
does that.  It starts mission_planner_node with the shipped config plus entered
waypoints along the course's own centreline: the start 15 m of lane ahead of
the robot, the end 10 m of lane after that, a 4 m gate.  It never calls any
service and never publishes /cmd_vel.

PASS needs, within 150 s of wall time:
  * /mission/state goes from idle to active by itself;
  * and the planner's goal is accepted by Nav2 (its log says
    "navigating to end waypoint" and no "rejected").
Also measured and printed, not required (driving a waypoint mission end to end
is course v2's job): whether the robot then reached the end waypoint (state
done or lane_follow after active) and how close it got.

The rules allow no operator during a run (2026 rules p.11), so before
2026-10-08, with auto_start false, the mission could only start by hand: on
that code this check FAILS (state stays idle).  Negative control: the same
check with PLANNER_PYTHONPATH pointing at the old code.
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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from generate_igvc_world import DATUM_LAT_DEG, DATUM_LON_DEG  # noqa: E402

REPO = os.environ.get('IGVC_WORKSPACE_ROOT', '/root/ros2_ws/src/IGVC_robot_2026')
TRACK = os.path.join(REPO, 'IGVC_track_generator', 'track_points.json')
ODOM = '/front_zed_camera_x/zed_node/odom'
OUT = os.environ.get('GPS_CHECK_OUT', '/tmp/gps_mission_check')
LIMIT_S = 150.0

A = 6378137.0
E2 = 6.69437999014e-3
phi = math.radians(DATUM_LAT_DEG)
M = A * (1 - E2) / (1 - E2 * math.sin(phi) ** 2) ** 1.5
N = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)


def world_to_latlon(x, y):
    return (DATUM_LAT_DEG + math.degrees(y / M),
            DATUM_LON_DEG + math.degrees(x / (N * math.cos(phi))))


track = json.load(open(TRACK, encoding='utf-8'))
C = [(p['x'], p['y']) for p in track['centerline_m']]
SPAWN = (track['robot_start_pose']['position_m']['x'],
         track['robot_start_pose']['position_m']['y'])


def along(start_idx, metres):
    """The centreline point `metres` of lane after index start_idx (the robot
    drives in increasing index from the start pose)."""
    d, i = 0.0, start_idx
    while d < metres:
        j = (i + 1) % len(C)
        d += math.hypot(C[j][0] - C[i][0], C[j][1] - C[i][1])
        i = j
    return C[i], i


S, si = along(0, 15.0)
E, ei = along(si, 10.0)
O1, _ = along(ei, 5.0)
O2, _ = along(ei, 7.0)

os.makedirs(OUT, exist_ok=True)
params = {
    'use_sim_time': True,
    'ramp_enabled': False,
    'gps_start_match_radius_m': 4.0,
    'waypoint_lats': [world_to_latlon(*p)[0] for p in (S, O1, O2, E)],
    'waypoint_lons': [world_to_latlon(*p)[1] for p in (S, O1, O2, E)],
    'waypoint_kinds': ['start', 'obstacle', 'obstacle', 'end'],
}
overlay_path = os.path.join(OUT, 'entered_waypoints.yaml')
yaml.safe_dump({'mission_planner_node': {'ros__parameters': params}},
               open(overlay_path, 'w', encoding='utf-8'))
shipped = os.path.join(get_package_share_directory('igvc_test_bringup'),
                       'config', 'mission_planner_config.yaml')

rclpy.init()
node = Node('gps_mission_check')
states, odom = [], {}
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def on_state(m):
    if not states or states[-1][1] != m.data:
        states.append((time.time(), m.data, odom.get('xy')))


node.create_subscription(String, '/mission/state', on_state, latched)
node.create_subscription(
    Odometry, ODOM,
    lambda m: odom.__setitem__('xy', (m.pose.pose.position.x, m.pose.pose.position.y)),
    qos_profile_sensor_data)

env = dict(os.environ)
if os.environ.get('PLANNER_PYTHONPATH'):
    env['PYTHONPATH'] = os.environ['PLANNER_PYTHONPATH'] + os.pathsep + env.get('PYTHONPATH', '')
log = open(os.path.join(OUT, 'planner.log'), 'w')
proc = subprocess.Popen(
    ['ros2', 'run', 'igvc_lane_detection', 'mission_planner_node', '--ros-args',
     '--params-file', shipped, '--params-file', overlay_path],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)

t0 = time.time()
while time.time() - t0 < LIMIT_S:
    rclpy.spin_once(node, timeout_sec=0.1)
    names = [s[1] for s in states]
    if 'active' in names and any(n in ('done', 'lane_follow') for n in names[names.index('active'):]):
        break
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
text = open(os.path.join(OUT, 'planner.log'), encoding='utf-8', errors='replace').read()

names = [s[1] for s in states]
started = 'active' in names
accepted = 'navigating to end waypoint' in text and 'NavigateToPose rejected' not in text
print('  start waypoint %.1f m of lane ahead, end 10 m after it, gate 4 m' % 15.0)
print('  /mission/state sequence: %s' % ' -> '.join(names) if names else
      '  /mission/state: nothing received')
for t, n, xy in states:
    if xy is not None:
        w = (SPAWN[0] + xy[0], SPAWN[1] + xy[1])
        print('    %-12s at +%5.1f s, robot %.1f m from the start waypoint, %.1f m from the end'
              % (n, t - t0, math.hypot(w[0] - S[0], w[1] - S[1]),
                 math.hypot(w[0] - E[0], w[1] - E[1])))
print('  started by itself (no service call): %s' % ('yes' if started else 'NO'))
print('  Nav2 accepted the end goal         : %s' % ('yes' if accepted else 'no'))
if started and any(n in ('done', 'lane_follow') for n in names[names.index('active'):]):
    xy = [s for s in states if s[1] in ('done', 'lane_follow')][-1][2]
    if xy is not None:
        w = (SPAWN[0] + xy[0], SPAWN[1] + xy[1])
        print('  (measured, not required) handed back with the robot %.1f m from the end waypoint'
              % math.hypot(w[0] - E[0], w[1] - E[1]))
ok = started and accepted
print()
print('  VERDICT : %s' % ('PASS' if ok else 'FAIL'))
sys.exit(0 if ok else 1)
