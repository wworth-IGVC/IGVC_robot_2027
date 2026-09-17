#!/usr/bin/env python3
# Count messages on the topics lane_segmentation_node consumes, for the
# cross-container DDS gate. Run the SAME script in both containers at once and
# compare: the Jazzy one is the reference, the Humble one is the thing under
# test. Counting beats `ros2 topic hz` because hz plus `timeout` plus pipefail
# reports a false failure when the timeout kills the producer.
#   docker exec igvc_gazebo             bash -lc "source /opt/ros/jazzy/setup.bash  && python3 topic_counter.py 30 JAZZY"
#   docker exec igvc_humble_fused_drive bash -lc "source /opt/ros/humble/setup.bash && python3 topic_counter.py 30 HUMBLE"
"""Count messages on the topics lane_segmentation_node consumes.

Counting, not `ros2 topic hz`: hz reports a rate computed from whatever it
managed to receive, and under `set -o pipefail` a timeout kill turns a
successful grep into a reported failure. A raw count over a known wall-clock
window cannot lie in either direction.
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

CAM = '/front_zed_camera_x/zed_node'
TOPICS = [
    (CAM + '/rgb/color/rect/image',        Image,      'rgb'),
    (CAM + '/depth/depth_registered',      Image,      'depth'),
    (CAM + '/rgb/color/rect/camera_info',  CameraInfo, 'camera_info'),
    (CAM + '/odom',                        Odometry,   'zed_odom'),
    ('/odom',                              Odometry,   'odom'),
    ('/clock',                             Clock,      'clock'),
    ('/tf',                                TFMessage,  'tf'),
    ('/tf_static',                         TFMessage,  'tf_static'),
]

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
label = sys.argv[2] if len(sys.argv) > 2 else '?'

rclpy.init()
n = Node('topic_counter')
counts = {name: 0 for _, _, name in TOPICS}
def mk(name):
    def cb(_m):
        counts[name] += 1
    return cb
for topic, typ, name in TOPICS:
    # sensor QoS (BEST_EFFORT) matches how a camera publisher is usually set
    # up; RELIABLE publishers still match a BEST_EFFORT subscriber.
    n.create_subscription(typ, topic, mk(name), qos_profile_sensor_data)

t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
el = time.time() - t0
print('COUNTS %s %.1f' % (label, el))
for _, _, name in TOPICS:
    print('  %-12s %6d   %6.2f Hz' % (name, counts[name], counts[name] / el))
n.destroy_node(); rclpy.shutdown()
