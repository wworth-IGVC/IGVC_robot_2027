#!/usr/bin/env python3
"""Save what the Hough lane detector saw at given SIMULATION times.

For each time T: the detector's own overlay (/lane_debug/cam0/overlay: thin
yellow = Hough segments, thick blue = fitted left line, thick red = fitted
right line, green = image centre), the raw front RGB image, the depth image
(.npy, metres), and a JSON file with the camera intrinsics K, the
base_link-from-camera transform, the odometry pose and the message stamps.
Enough to replay the detector's projection and grid offline.

Run INSIDE the Gazebo container, beside an autonomy run in camera mode:

  python3 scripts/gazebo/measurements/grab_frames.py /tmp/grab 16 18 20 &
  LAUNCH_ARGS="lane_source:=hough" bash scripts/gazebo/autonomy_check.sh

It exits after the last time. Writes OUTDIR/t<T>_overlay.png, _rgb.png,
_depth.npy and _meta.json. Publishes nothing.
"""
import json
import os
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

OVERLAY = '/lane_debug/cam0/overlay'
RGB = '/front_zed_camera_x/zed_node/rgb/color/rect/image'
INFO = '/front_zed_camera_x/zed_node/rgb/color/rect/camera_info'


class Grab(Node):
    def __init__(self, out, times):
        super().__init__('frame_grabber', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.out, self.times, self.br = out, sorted(times), CvBridge()
        self.last = {}
        self.depth_topic = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(Image, OVERLAY, lambda m: self._keep('overlay', m), 5)
        self.create_subscription(Image, RGB, lambda m: self._keep('rgb', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, INFO, lambda m: self._keep('info', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self._keep('odom', m), 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_timer(2.0, self._find_depth)

    def _find_depth(self):
        if self.depth_topic:
            return
        for name, types in self.get_topic_names_and_types():
            if name.startswith('/front_zed_camera_x') and 'depth' in name and 'sensor_msgs/msg/Image' in types:
                self.depth_topic = name
                self.create_subscription(Image, name, lambda m: self._keep('depth', m), qos_profile_sensor_data)
                print('depth topic', name, flush=True)
                return

    def _keep(self, k, m):
        self.last[k] = m

    def _on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        while self.times and t >= self.times[0]:
            self._save(self.times.pop(0))
        if not self.times:
            raise SystemExit

    def _save(self, t):
        tag = os.path.join(self.out, 't%05.1f' % t)
        meta = {'sim_t': t, 'have': sorted(self.last)}
        try:
            if 'overlay' in self.last:
                cv2.imwrite(tag + '_overlay.png', self.br.imgmsg_to_cv2(self.last['overlay'], 'bgr8'))
                s = self.last['overlay'].header.stamp
                meta['overlay_stamp'] = s.sec + s.nanosec * 1e-9
            if 'rgb' in self.last:
                cv2.imwrite(tag + '_rgb.png', self.br.imgmsg_to_cv2(self.last['rgb'], 'bgr8'))
                s = self.last['rgb'].header.stamp
                meta['rgb_stamp'] = s.sec + s.nanosec * 1e-9
                meta['rgb_frame'] = self.last['rgb'].header.frame_id
            if 'depth' in self.last:
                d = self.last['depth']
                np.save(tag + '_depth.npy', self.br.imgmsg_to_cv2(d, d.encoding))
                meta['depth_encoding'] = d.encoding
                meta['depth_topic'] = self.depth_topic
            if 'info' in self.last:
                meta['K'] = list(self.last['info'].k)
                meta['size'] = [self.last['info'].width, self.last['info'].height]
            if 'odom' in self.last:
                p = self.last['odom'].pose.pose
                meta['odom'] = [p.position.x, p.position.y, p.orientation.z, p.orientation.w]
            fr = meta.get('rgb_frame')
            if fr:
                tr = self.tf.lookup_transform('base_link', fr, rclpy.time.Time())
                q, v = tr.transform.rotation, tr.transform.translation
                meta['base_from_cam'] = [v.x, v.y, v.z, q.x, q.y, q.z, q.w]
        except Exception as e:  # keep going; record what failed
            meta['error'] = repr(e)
        json.dump(meta, open(tag + '_meta.json', 'w'), indent=1)
        print('saved', t, meta.get('have'), meta.get('error', ''), flush=True)


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = Grab(out, [float(a) for a in sys.argv[2:]])
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    print('done', flush=True)


if __name__ == '__main__':
    main()
