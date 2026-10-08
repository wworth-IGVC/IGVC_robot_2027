"""
gazebo_cloud_restamp.py

Give the simulated camera point cloud the frame id its data is really in.

THE PROBLEM, measured rather than assumed
-----------------------------------------
Gazebo's rgbd_camera publishes four outputs (image, depth, camera_info and the
point cloud) and one <gz_frame_id> in gazebo_sim.urdf.xacro stamps all four.
That tag names the OPTICAL frame, <cam>_left_camera_frame_optical, which is
right for the image, the depth and camera_info: their intrinsics are optical by
REP 145, and pinhole projection depends on it.

It is wrong for the cloud. gz-sensors emits the points in the BODY convention
(x forward, z up), and gz_frame_id only sets a string; it does not rotate data.
So TF rotates the points a second time, by 90 deg about two axes, and the floor
comes out as a wall in front of the robot. cloud_frame_check.py measures this:
on the raw cloud the dominant plane's normal is on base_link X at -0.096 m
(its case C) instead of on Z at -0.229 m. See docs/GAZEBO_SETUP.md 10.6.

THE FIX
-------
The bridge delivers the cloud to a private name, and this node republishes it
on the contract name with header.frame_id set to the body frame,
<cam>_left_camera_frame. The data is not touched: same points, same stamp,
same fields. One tag cannot carry two frames, and the image, depth and
camera_info must stay optical, so the cloud is the only thing re-stamped.

The real ZED wrapper publishes its cloud in that body frame too (the
competition branch's lane_segmentation.py says so where it projects lane pixels
through the cloud), so this makes the simulated cloud match the real one, not
just look right in RViz.

SPEED: THE BYTES ARE EDITED, NOT THE MESSAGE
-------------------------------------------
The first version deserialised each cloud, changed the string and published
it again. Measured 2026-10-07 at 640x360: the bridge delivered 9.8 Hz and that
node republished 1.6 Hz (16.5%), because rclpy converts all 230400 points to
Python and back. So this node subscribes to the SERIALIZED message
(raw=True), replaces the frame_id string inside the CDR bytes, and publishes
the bytes. The point data is copied once, never decoded.

IT DROPS NOTHING
----------------
The node logs, every 10 s, the clouds it received and the clouds it published.
They are equal in every report. A best-effort subscriber to its output once saw
only 2.0 to 3.8 Hz of the 7 to 13 Hz it was publishing, and that loss was in
the transport, not here: a 3.7 MB cloud crosses the UDP-only loopback as a burst
that overflows the reader's socket buffer. The cure is the Fast DDS flow
controller in config/fastdds_sim_cloud_pacing.xml, which gazebo_sim.launch.py
gives to this node and to the bridge. Without it this node is exactly as
correct and a consumer receives a fraction of the clouds.
docs/GAZEBO_SETUP.md 10.13.

The layout it relies on (CDR, as rmw writes a PointCloud2): a 4-byte
encapsulation header, header.stamp (int32 sec, uint32 nanosec), then
header.frame_id as a uint32 length (counting the terminating NUL) and the
characters. Everything after the string is aligned to 4 bytes at most, so
moving it by a multiple of 4 keeps it valid; the string is padded to make
that true. The first cloud is decoded both before and after the edit and
compared field by field, and the node refuses to publish if anything other
than the frame_id differs, so a layout surprise stops it loudly instead of
publishing garbage.

QOS
---
The subscription is best effort on purpose. A reliable reader that falls
behind can make the bridge's writer wait, and the bridge carries every other
sensor and the clock. Dropping a cloud is better than slowing those.

PARAMETERS
----------
    input_topic    str   /sim/front_zed/points_optical_stamp
    output_topic   str   /front_zed_camera_x/zed_node/point_cloud/cloud_registered
    frame_id       str   front_zed_camera_x_left_camera_frame
"""

from __future__ import annotations

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def _align4(offset: int) -> int:
    """Round up to 4, counting from the end of the encapsulation header."""
    return 4 + ((offset - 4 + 3) // 4) * 4


def restamp_cdr(raw: bytes, frame_id: bytes) -> bytes:
    """Return the serialized message with header.frame_id replaced."""
    if raw[0] != 0 or raw[1] not in (0, 1):
        raise ValueError('not a plain CDR message: %r' % raw[:4])
    endian = '<' if raw[1] == 1 else '>'
    (old_len,) = struct.unpack_from(endian + 'I', raw, 12)
    old_end = _align4(16 + old_len)
    name = frame_id + b'\0'
    new_head = raw[:12] + struct.pack(endian + 'I', len(name)) + name
    new_head += b'\0' * (_align4(len(new_head)) - len(new_head))
    return new_head + raw[old_end:]


class GazeboCloudRestamp(Node):

    def __init__(self) -> None:
        super().__init__('gazebo_cloud_restamp')

        self.declare_parameter('input_topic',
                               '/sim/front_zed/points_optical_stamp')
        self.declare_parameter(
            'output_topic',
            '/front_zed_camera_x/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('frame_id',
                               'front_zed_camera_x_left_camera_frame')

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        self._frame = str(self.get_parameter('frame_id').value)
        self._frame_b = self._frame.encode('utf-8')

        # Matches what parameter_bridge offers by default, so reliable
        # consumers such as Nav2's obstacle layer can still connect.
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(PointCloud2, out_topic, out_qos)
        self.create_subscription(PointCloud2, in_topic, self._on_cloud,
                                 qos_profile_sensor_data, raw=True)

        self._checked = False
        # Counted inside the node so a loss can be put on the receive side or
        # the publish side, without trusting an outside subscriber (a counter
        # that must itself take 3.7 MB messages can lose them too).
        self._n_in = 0
        self._n_out = 0
        self._t_report = time.monotonic()   # wall time, as the counters use
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            'Gazebo cloud re-stamp ready: %s -> %s, frame_id %s'
            % (in_topic, out_topic, self._frame))

    def _check_once(self, raw: bytes, out: bytes) -> None:
        """Decode the first cloud before and after; only frame_id may differ."""
        a = deserialize_message(raw, PointCloud2)
        b = deserialize_message(out, PointCloud2)
        same = (a.header.stamp == b.header.stamp
                and a.height == b.height and a.width == b.width
                and a.fields == b.fields
                and a.is_bigendian == b.is_bigendian
                and a.point_step == b.point_step
                and a.row_step == b.row_step
                and a.is_dense == b.is_dense
                and bytes(a.data) == bytes(b.data)
                and b.header.frame_id == self._frame)
        if not same:
            raise RuntimeError(
                'the CDR edit changed more than frame_id; refusing to '
                'publish. See the module docstring.')
        self.get_logger().info(
            'first cloud checked: %s -> %s, %d x %d points, data identical'
            % (a.header.frame_id, b.header.frame_id, a.width, a.height))
        self._checked = True

    def _report(self) -> None:
        now = time.monotonic()
        dt = now - self._t_report
        self._t_report = now
        self.get_logger().info(
            'clouds in %d (%.2f Hz), out %d (%.2f Hz) over %.1f s'
            % (self._n_in, self._n_in / dt, self._n_out, self._n_out / dt, dt))
        self._n_in = self._n_out = 0

    def _on_cloud(self, raw: bytes) -> None:
        self._n_in += 1
        out = restamp_cdr(raw, self._frame_b)
        if not self._checked:
            self._check_once(raw, out)
        self._pub.publish(out)
        self._n_out += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboCloudRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
