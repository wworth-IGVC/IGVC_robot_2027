#!/usr/bin/env python3
# Measure whether the RGB and depth streams are close enough in time for
# lane_detection.py:88-89 ApproximateTimeSynchronizer(queue_size=5, slop=0.1)
# to fire. Subscribes to BOTH at once, which is the only valid way: two
# sequential `ros2 topic echo --once` calls recover their own sampling interval
# and nothing else. Run inside the Gazebo container with the sim up.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

RGB = '/front_zed_camera_x/zed_node/rgb/color/rect/image'
DEP = '/front_zed_camera_x/zed_node/depth/depth_registered'

class P(Node):
    def __init__(self):
        super().__init__('skew_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.rgb, self.dep, self.hits = [], [], {0.1: 0, 0.3: 0, 1.0: 0}
        self.create_subscription(Image, RGB, lambda m: self.rgb.append(self._t(m)), 10)
        self.create_subscription(Image, DEP, lambda m: self.dep.append(self._t(m)), 10)
        # exactly what lane_detection.py:88-89 builds, plus two looser slops
        for slop in (0.1, 0.3, 1.0):
            s = ApproximateTimeSynchronizer(
                [Subscriber(self, Image, RGB), Subscriber(self, Image, DEP)],
                queue_size=5, slop=slop)
            s.registerCallback(lambda a, b, sl=slop: self.hits.__setitem__(sl, self.hits[sl] + 1))
    @staticmethod
    def _t(m):
        return m.header.stamp.sec + m.header.stamp.nanosec / 1e9

rclpy.init()
n = P()
import time
t0 = time.time()
while time.time() - t0 < 30.0:
    rclpy.spin_once(n, timeout_sec=0.2)

print('  rgb msgs   : %d' % len(n.rgb))
print('  depth msgs : %d' % len(n.dep))
if n.rgb and n.dep:
    # for each rgb stamp, the closest depth stamp
    d = [min(abs(r - x) for x in n.dep) for r in n.rgb]
    d.sort()
    print('  nearest-depth stamp gap: min %.4f  median %.4f  max %.4f s'
          % (d[0], d[len(d)//2], d[-1]))
    print('  rgb stamp span  : %.2f s over %d msgs' % (n.rgb[-1]-n.rgb[0], len(n.rgb)))
    print('  depth stamp span: %.2f s over %d msgs' % (n.dep[-1]-n.dep[0], len(n.dep)))
print('  synchroniser fires:')
for k in sorted(n.hits):
    print('    slop=%.1f s -> %d callbacks%s' % (k, n.hits[k], '   <-- lane_detection.py uses this' if k == 0.1 else ''))
n.destroy_node(); rclpy.shutdown()
