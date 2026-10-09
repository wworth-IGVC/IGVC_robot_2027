#!/usr/bin/env python3
"""
camera_rate_check.py

Does a BEST-EFFORT subscriber receive the simulated camera's RGB image, depth
image or point cloud, or only a fraction of it?  The same test as
cloud_rate_check.py, for any of the three: one large stream per run (a second
large subscriber in the same process starves itself, docs/GAZEBO_SETUP.md
10.13), counted against the camera's own per-frame camera_info, with the
simulation clock alongside.

    python3 camera_rate_check.py --stream image|depth|cloud [--seconds 45]
                                 [--camera front] [--hz 15]

PASS means the stream arrived at 95% or better of the camera_info count.
Subscriptions are qos_profile_sensor_data (best effort), how the competition's
lane_segmentation subscribes.  Counting messages, never `ros2 topic hz`.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

ap = argparse.ArgumentParser()
ap.add_argument('--stream', required=True, choices=('image', 'depth', 'cloud'))
ap.add_argument('--seconds', type=float, default=45.0)
ap.add_argument('--camera', default='front', choices=('front', 'left', 'right'))
ap.add_argument('--hz', type=float, default=15.0)
args = ap.parse_args()

ZED = '/%s_zed_camera_x/zed_node' % args.camera
TOPIC, TYPE = {
    'image': (ZED + '/rgb/color/rect/image', Image),
    'depth': (ZED + '/depth/depth_registered', Image),
    'cloud': (ZED + '/point_cloud/cloud_registered', PointCloud2),
}[args.stream]
INFO = ZED + '/rgb/color/rect/camera_info'

rclpy.init()
node = Node('camera_rate_check_%s' % args.stream)
count = {'stream': 0, 'info': 0}
size = {}
clk = {}


def on_stream(msg):
    count['stream'] += 1
    size['bytes'] = len(msg.data)


def on_info(msg):
    count['info'] += 1


def on_clock(msg):
    t = msg.clock.sec + msg.clock.nanosec * 1e-9
    clk.setdefault('first', t)
    clk['last'] = t


node.create_subscription(TYPE, TOPIC, on_stream, qos_profile_sensor_data)
node.create_subscription(CameraInfo, INFO, on_info, qos_profile_sensor_data)
node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)

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
ratio = count['stream'] / count['info'] if count['info'] else 0.0
ok = count['info'] > 0 and ratio >= 0.95
print('  %-6s %-58s %5d msgs %6.2f Hz  (%.2f MB each)'
      % (args.stream, TOPIC, count['stream'], count['stream'] / wall,
         size.get('bytes', 0) / 1e6))
print('  camera_info %5d msgs, %.0f by the clock (%.1f Hz x %.1f s sim), RTF %.2f'
      % (count['info'], args.hz * sim, args.hz, sim, sim / wall if wall else 0.0))
print('  RATE   : %s  %s/camera_info = %.3f (need >= 0.95)'
      % ('PASS' if ok else 'FAIL', args.stream, ratio))
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
