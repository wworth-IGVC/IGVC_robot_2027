#!/usr/bin/env python3
"""Does /fix put the GPS antenna where Gazebo says it is?

Run inside the simulator's container with gazebo_sim.launch.py up (nothing
else driving the robot). It takes a fix with the robot at rest, drives it
straight at 0.5 m/s for 3 s of wall time, stops it, and takes another. Each
fix is converted to world metres through the world's datum (generate_igvc_world.py: ENU, heading 0, so
+x east and +y north) and compared with the antenna's ground-truth position:
`gz model -p` for the robot, plus the base_link -> front_gps_antenna_link
offset from TF, rotated by the robot's yaw.

PASS needs all of:
  * both fixes within 0.05 m of ground truth (the sensor has no noise)
  * the displacement between the fixes within 1 deg of ground truth's
  * frame_id front_gps_antenna_link, the u-blox driver's frame

The negative control is a wrong assumption about the world's axes:
    fix_check.py --rotate-deg 90
rotates the fix-derived positions by 90 deg about the datum before comparing.
It MUST fail. If it passes, the comparison cannot see a heading error.

The datum is a PLACEHOLDER, so this proves the chain, not the venue.
"""
import math
import os
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from generate_igvc_world import DATUM_LAT_DEG, DATUM_LON_DEG  # noqa: E402

ROBOT = 'igvc_robot'
FRAME = 'front_gps_antenna_link'
POS_TOL_M = 0.05
HEADING_TOL_DEG = 1.0

rotate_deg = 0.0
if sys.argv[1:2] == ['--rotate-deg']:
    rotate_deg = float(sys.argv[2])

# WGS84 local radii at the datum. Over tens of metres the error of this
# linearisation is well under a millimetre.
A = 6378137.0
E2 = 6.69437999014e-3
phi = math.radians(DATUM_LAT_DEG)
M = A * (1 - E2) / (1 - E2 * math.sin(phi) ** 2) ** 1.5   # meridian
N = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)             # prime vertical


def fix_to_world(lat, lon):
    east = math.radians(lon - DATUM_LON_DEG) * N * math.cos(phi)
    north = math.radians(lat - DATUM_LAT_DEG) * M
    r = math.radians(rotate_deg)
    return (math.cos(r) * east - math.sin(r) * north,
            math.sin(r) * east + math.cos(r) * north)


def gz_pose():
    out = subprocess.run(['gz', 'model', '-m', ROBOT, '-p'],
                         capture_output=True, text=True, timeout=30).stdout
    vecs = re.findall(r'\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]',
                      out[out.find('Pose'):])
    if len(vecs) < 2:
        print('  FAIL: could not read the robot pose from gz model:\n' + out)
        sys.exit(2)
    (x, y, _), (_, _, yaw) = [tuple(map(float, v)) for v in vecs[:2]]
    return x, y, yaw


rclpy.init()
n = Node('fix_check')
buf = Buffer()
TransformListener(buf, n)
latest = {}
n.create_subscription(NavSatFix, '/fix',
                      lambda m: latest.__setitem__('m', m),
                      qos_profile_sensor_data)
cmd = n.create_publisher(Twist, '/cmd_vel', 10)


def spin_for(seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(n, timeout_sec=0.05)


def fresh_fix(limit=30.0):
    latest.pop('m', None)
    t0 = time.time()
    while 'm' not in latest and time.time() - t0 < limit:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' not in latest:
        print('  FAIL: no /fix in %.0f s' % limit)
        sys.exit(2)
    return latest['m']


def send(vx):
    t = Twist()
    t.linear.x = vx
    cmd.publish(t)


tf = None
t0 = time.time()
while tf is None and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        tf = buf.lookup_transform('base_link', FRAME, rclpy.time.Time())
    except Exception:
        pass
if tf is None:
    print('  FAIL: no TF base_link <- %s' % FRAME)
    sys.exit(2)
ox, oy = tf.transform.translation.x, tf.transform.translation.y
print('  antenna offset      : x %+.3f y %+.3f m from base_link' % (ox, oy))
print('  datum (placeholder) : %.9f, %.9f' % (DATUM_LAT_DEG, DATUM_LON_DEG))
if rotate_deg:
    print('  CONTROL RUN         : fixes rotated %.0f deg before comparing'
          % rotate_deg)


def sample(label):
    # Stationary: settle, then take a fix and the pose together.
    for _ in range(10):
        send(0.0)
        spin_for(0.1)
    spin_for(2.0)
    m = fresh_fix()
    x, y, yaw = gz_pose()
    ax = x + math.cos(yaw) * ox - math.sin(yaw) * oy
    ay = y + math.sin(yaw) * ox + math.cos(yaw) * oy
    fx, fy = fix_to_world(m.latitude, m.longitude)
    err = math.hypot(fx - ax, fy - ay)
    print('  %-6s fix -> world  : (%+.3f, %+.3f)   truth (%+.3f, %+.3f)   '
          'error %.3f m   frame %s'
          % (label, fx, fy, ax, ay, err, m.header.frame_id))
    return (fx, fy), (ax, ay), err, m.header.frame_id


f1, t1, e1, fr1 = sample('start')
t_drive = time.time()
while time.time() - t_drive < 3.0:
    send(0.5)
    spin_for(0.1)
f2, t2, e2, fr2 = sample('end')

dfx, dfy = f2[0] - f1[0], f2[1] - f1[1]
dtx, dty = t2[0] - t1[0], t2[1] - t1[1]
dist = math.hypot(dtx, dty)
dh = math.degrees(math.atan2(dfy, dfx) - math.atan2(dty, dtx))
dh = (dh + 180.0) % 360.0 - 180.0
print('  displacement        : truth %.3f m, heading difference %+.2f deg'
      % (dist, dh))

ok = True
if max(e1, e2) > POS_TOL_M:
    print('  POSITION : FAIL  worst error %.3f m (limit %.2f)'
          % (max(e1, e2), POS_TOL_M)); ok = False
else:
    print('  POSITION : PASS  worst error %.3f m' % max(e1, e2))
if dist < 0.5:
    print('  HEADING  : FAIL  the robot moved only %.2f m, too little to judge'
          % dist); ok = False
elif abs(dh) > HEADING_TOL_DEG:
    print('  HEADING  : FAIL  %+.2f deg (limit %.1f)' % (dh, HEADING_TOL_DEG))
    ok = False
else:
    print('  HEADING  : PASS  %+.2f deg' % dh)
if fr1 != FRAME or fr2 != FRAME:
    print('  FRAME    : FAIL  %s, expected %s' % (fr1, FRAME)); ok = False
else:
    print('  FRAME    : PASS  %s' % FRAME)
print('  VERDICT  : %s' % ('PASS' if ok else 'FAIL'))
n.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
