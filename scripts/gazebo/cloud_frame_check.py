#!/usr/bin/env python3
"""P0-3: is the camera rgbd point cloud oriented the way its frame_id claims?

All four rgbd_camera outputs are stamped with the OPTICAL frame via one
gz_frame_id. That is right for the image and depth rasters, whose intrinsics are
optical by REP 145. It is only right for the CLOUD if gz-sensors emits the
points in the optical convention too. gz_frame_id sets a string; it does not
rotate data. If the cloud is body-convention but labelled optical, TF applies the
90/90 rotation a second time, the floor becomes a wall, and Nav2's obstacle_layer
marks the ground lethal.

The four cases are 90 degrees apart, so this is not a marginal measurement:

  A  optical data, stamped optical (CORRECT)  normal +Z, plane at z = -0.229
  B  optical data, BODY frame forced          normal +Y, plane at y = +0.551
  C  body data, stamped optical (THE BUG)     normal +X, plane at x = -0.096
  D  body data, BODY frame forced             normal +Z, plane at z = -0.229

Run with no argument for the real test, and with the body frame name as the
control: the control MUST fail, and its signature says which case holds.
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

TOPIC = '/front_zed_camera_x/zed_node/point_cloud/cloud_registered'
override = sys.argv[1] if len(sys.argv) > 1 else None

rclpy.init()
n = Node('cloud_check')
buf = Buffer(); TransformListener(buf, n)
got = {}
n.create_subscription(PointCloud2, TOPIC,
                      lambda m: got.setdefault('m', m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 30 and 'm' not in got:
    rclpy.spin_once(n, timeout_sec=0.1)
if 'm' not in got:
    print('  FAIL: no cloud in 30 s'); sys.exit(2)
m = got['m']
src = override or m.header.frame_id
print('  stamped frame       : %s' % m.header.frame_id)
if override:
    print('  FORCED through      : %s   <-- control run' % override)

# Jazzy's read_points returns a STRUCTURED array, so index by field name.
arr = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)
# skip_nans does NOT drop infinities, and gazebo_bridge.yaml:146-151 warns the
# depth is exact ray-cast range with +inf past the far clip. Filter explicitly,
# or the plane fit silently works on a cloud whose extent is inf.
raw = len(pts)
pts = pts[np.isfinite(pts).all(axis=1)]
print('  finite points       : %d of %d (%d non-finite dropped)'
      % (len(pts), m.width * m.height, raw - len(pts)))
if len(pts) < 500:
    print('  FAIL: too few finite points'); sys.exit(2)

tf = None
deadline = time.time() + 15
while time.time() < deadline:
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        tf = buf.lookup_transform('base_link', src, rclpy.time.Time()); break
    except Exception:
        continue
if tf is None:
    print('  FAIL: no TF base_link <- %s' % src); sys.exit(2)

q, t = tf.transform.rotation, tf.transform.translation
x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
              [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
P = pts @ R.T + np.array([t.x, t.y, t.z])
print('  base_link extent    : x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]'
      % (P[:,0].min(), P[:,0].max(), P[:,1].min(), P[:,1].max(),
         P[:,2].min(), P[:,2].max()))

rng = np.random.default_rng(0)
S = P[rng.choice(len(P), size=min(len(P), 20000), replace=False)]
best = (-1, None, None)
for _ in range(400):
    a, b, c = S[rng.choice(len(S), 3, replace=False)]
    nv = np.cross(b - a, c - a); nl = np.linalg.norm(nv)
    if nl < 1e-9: continue
    nv /= nl; d = -nv @ a
    inl = int(np.sum(np.abs(S @ nv + d) < 0.03))
    if inl > best[0]: best = (inl, nv, d)
inl, nv, d = best
if nv[2] < 0 and abs(nv[2]) > 0.5: nv, d = -nv, -d
axis = 'XYZ'[int(np.argmax(np.abs(nv)))]
ang = math.degrees(math.acos(min(1.0, abs(float(nv @ np.array([0, 0, 1.0]))))))
off = -d / nv[int(np.argmax(np.abs(nv)))]
print('  dominant plane      : normal [%+.3f %+.3f %+.3f], %.0f%% inliers'
      % (nv[0], nv[1], nv[2], 100.0*inl/len(S)))
print('  normal axis         : %s        offset on that axis: %+.4f m' % (axis, off))
print('  angle to base_link +Z : %.2f deg' % ang)

if axis == 'Z' and abs(off + 0.2290) < 0.05:
    verdict = 'PASS  optical data, stamped optical: gz_frame_id is CORRECT'
    rc = 0
elif axis == 'X' and abs(off + 0.0960) < 0.08:
    verdict = 'FAIL  body data stamped optical: THE CLOUD IS ROTATED 90 deg'
    rc = 1
elif axis == 'Y' and abs(off - 0.5509) < 0.08:
    verdict = 'FAIL(expected for the control) optical data forced through body frame'
    rc = 1
elif axis == 'Z' and abs(off + 0.4910) < 0.08:
    verdict = 'FAIL  plane at camera height: frame chain wrong'
    rc = 1
else:
    verdict = 'FAIL  unrecognised signature; transform path is broken'
    rc = 1
print('  VERDICT             : %s' % verdict)
n.destroy_node(); rclpy.shutdown(); sys.exit(rc)
