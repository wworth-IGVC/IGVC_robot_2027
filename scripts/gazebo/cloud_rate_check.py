#!/usr/bin/env python3
"""
cloud_rate_check.py

Does a BEST-EFFORT subscriber receive the simulated point cloud, or only a
fraction of it?

It subscribes to the cloud consumers read,
/<cam>_zed_camera_x/zed_node/point_cloud/cloud_registered, the way the
competition's lane_segmentation does (qos_profile_sensor_data, best effort),
and counts messages for a wall-clock window. It compares that with the number
of frames the camera produced in the same window, which it gets two ways that
do not involve a second 3.7 MB subscription:

  camera_info   one small message per camera frame, counted in the window
  clock         the camera's update rate times the simulation seconds that
                passed (the rate is --hz, default 15, sim_camera_hz)

PASS means the cloud arrived at 95% or better of the camera_info count.

WHY NOT COUNT THE RAW CLOUD AS "IN"
Measured 2026-10-07: a second 3.7 MB subscriber in the same process starved
itself. With the cloud paced and the re-stamped cloud arriving at 100% of what
the camera produced, the raw cloud counted by that same process read 0.70 Hz,
and an out/raw ratio of 17.8 "passed". The raw cloud is therefore printed for
information only, and only when asked for (--raw).

WHY COUNT MESSAGES
Not `ros2 topic hz`, and not the real-time factor: a 3.7 MB message is lost
whole to a single dropped UDP datagram while the simulator reports a healthy
RTF. That is what happened before config/fastdds_sim_camera_pacing.xml (2.0 to
3.8 Hz received while the re-stamp node was publishing 7 to 13 Hz), and this is
the check that sees it. See docs/GAZEBO_SETUP.md 10.13.

    python3 cloud_rate_check.py [--seconds 60] [--camera front] [--hz 15] [--raw]

Exit 0 on PASS, 1 on FAIL. Needs a running simulator with that camera
(sim_cameras:=front, the default, spawns only the front one).
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, PointCloud2

ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=60.0)
ap.add_argument('--camera', default='front', choices=('front', 'left', 'right'))
ap.add_argument('--hz', type=float, default=15.0,
                help="the camera's update rate (sim_camera_hz)")
ap.add_argument('--raw', action='store_true',
                help='also count the raw cloud, for information only')
args = ap.parse_args()

ZED = '/%s_zed_camera_x/zed_node' % args.camera
OUT = ZED + '/point_cloud/cloud_registered'
INFO = ZED + '/rgb/color/rect/camera_info'
RAW = '/sim/%s_zed/points_optical_stamp' % args.camera

rclpy.init()
node = Node('cloud_rate_check')
count = {'out': 0, 'info': 0, 'raw': 0}
frame = {}
clk = {}


def counter(key):
    def cb(msg):
        count[key] += 1
        frame[key] = msg.header.frame_id
    return cb


def on_clock(msg):
    t = msg.clock.sec + msg.clock.nanosec * 1e-9
    clk.setdefault('first', t)
    clk['last'] = t


node.create_subscription(PointCloud2, OUT, counter('out'), qos_profile_sensor_data)
node.create_subscription(CameraInfo, INFO, counter('info'), qos_profile_sensor_data)
node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
if args.raw:
    node.create_subscription(PointCloud2, RAW, counter('raw'), qos_profile_sensor_data)

# Let discovery finish before the window opens.
t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
for k in count:
    count[k] = 0
clk.clear()

t0 = time.time()
while time.time() - t0 < args.seconds:
    rclpy.spin_once(node, timeout_sec=0.05)
wall = time.time() - t0
sim = clk.get('last', 0.0) - clk.get('first', 0.0)
expected = args.hz * sim

print('  window      : %.1f s wall, %.1f s sim, real-time factor %.2f'
      % (wall, sim, sim / wall if wall else 0.0))
print('  camera_info : %5d msgs  %.2f Hz  (frames the camera produced)'
      % (count['info'], count['info'] / wall))
print('  by the clock: %5.0f frames  (%.1f Hz x %.1f s sim)' % (expected, args.hz, sim))
print('  cloud       : %5d msgs  %.2f Hz  frame %s'
      % (count['out'], count['out'] / wall, frame.get('out')))
if args.raw:
    print('  raw cloud   : %5d msgs  %.2f Hz  (information only: a second large '
          'subscriber starves itself)' % (count['raw'], count['raw'] / wall))
ratio = count['out'] / count['info'] if count['info'] else 0.0
ok = count['info'] > 0 and ratio >= 0.95
print('  RATE        : %s  cloud/camera_info = %.3f (need >= 0.95), cloud/clock = %.3f'
      % ('PASS' if ok else 'FAIL', ratio, count['out'] / expected if expected else 0.0))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
