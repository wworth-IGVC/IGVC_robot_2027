"""
gazebo_odom_shim.py

Make Gazebo's odometry mean what the IGVC stack thinks odometry means.

THE PROBLEM, measured rather than assumed
-----------------------------------------
Gazebo's DiffDrive system starts its odometry at identity, so the odom frame's
axes are the robot's heading *at spawn*. The IGVC course spawns the robot at
world (-11.8817, 0.4511) facing 134.87 deg, and a run on 2026-09-15 read
/odom as (0.0000, 0.0000) yaw 0.00 deg at that instant. Driving forward for
4 s moved the robot 3.03 m through the world along 134.87 deg while the raw
odometry recorded 2.85 m along its own +x.

The 2026 code assumes the opposite. track_ground_truth_node._track_to_map says
so in as many words:

    # Odom/map frame is world-axis aligned with the spawn point at (0, 0).
    # Only translate by -origin_offset; Isaac Sim's odom inherits world axes
    # from the spawn pose, so the map must NOT be rotated by start yaw.

and it builds the whole ground-truth occupancy grid on that basis. Worse,
gt_nav_bridge_node reads the odometry pose as a pose in the ground-truth grid's
frame with no TF lookup anywhere, so nothing in the TF tree can rescue it: feed
it raw Gazebo odometry and the local costmap is cropped from a point rotated
134.87 deg away from the robot, every time, silently, with all topics green.

That is a whole class of bug this project has been bitten by before, so it gets
a node rather than a comment.

WHY THIS IS A SIMULATOR DIFFERENCE AND NOT A GAZEBO BUG
-------------------------------------------------------
Gazebo is right about the real world. Wheel odometry on the physical robot also
starts at identity wherever the robot booted, because there is no world frame
to inherit axes from; that is what map->odom exists to absorb. Isaac Sim was
the odd one out: it published a ground-truth pose that happened to carry world
axes, and the stack was written against that convenience. This node reproduces
the convenience so nothing downstream has to change.

THE TRANSFORM
-------------
Let O be Gazebo's odom frame (origin at the spawn point, axes along the spawn
heading) and M the frame the stack expects (origin at the spawn point, world
axes). Both share an origin, so the transform is a pure rotation by the spawn
yaw:

    p_M = Rot(yaw_spawn) * p_O
    q_M = Rot(yaw_spawn) * q_O

The twist is NOT rotated. REP 103 puts Odometry twist in the child frame
(base_link), and base_link is unaffected by how the parent frame is oriented.
Rotating it here would be a real bug and would show up as a robot that
believes it is crabbing sideways.

PUBLICATIONS
------------
    /odom                                     nav_msgs/Odometry   corrected
    /front_zed_camera_x/zed_node/odom         nav_msgs/Odometry   contract name
    /tf  (odom -> base_link)                  corrected

The contract name is section 3.7's default odom_topic, the one
isaac_nav_test.launch.py remaps every consumer onto. Publishing both means the
Gazebo nav test can use exactly the remapping the Isaac one uses, which is how
we know the rename works rather than assuming it.

Gazebo's own TF output is deliberately not bridged - see gazebo_bridge.yaml.
This node is the only publisher of odom -> base_link.

PARAMETERS
----------
    spawn_yaw_rad     float   0.0   the yaw the robot was spawned at
    input_topic       str     /sim/odom_body_frame
    odom_topic        str     /odom
    zed_odom_topic    str     /front_zed_camera_x/zed_node/odom
    odom_frame_id     str     odom
    base_frame_id     str     base_link
    publish_tf        bool    True
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def _quat_mul(a, b):
    """Hamilton product (x, y, z, w) * (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class GazeboOdomShim(Node):

    def __init__(self) -> None:
        super().__init__('gazebo_odom_shim')

        self.declare_parameter('spawn_yaw_rad', 0.0)
        self.declare_parameter('input_topic', '/sim/odom_body_frame')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('zed_odom_topic',
                               '/front_zed_camera_x/zed_node/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self._yaw = float(self.get_parameter('spawn_yaw_rad').value)
        self._odom_frame = str(self.get_parameter('odom_frame_id').value)
        self._base_frame = str(self.get_parameter('base_frame_id').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        # Half-angle rotation quaternion about +z, applied on the left.
        self._rot = (0.0, 0.0, math.sin(self._yaw / 2.0),
                     math.cos(self._yaw / 2.0))
        self._cos = math.cos(self._yaw)
        self._sin = math.sin(self._yaw)

        # Sensor-ish data: keep the newest, do not block on a slow consumer.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), qos)
        self._zed_pub = self.create_publisher(
            Odometry, str(self.get_parameter('zed_odom_topic').value), qos)
        self._tf = TransformBroadcaster(self) if self._publish_tf else None

        in_topic = str(self.get_parameter('input_topic').value)
        self.create_subscription(Odometry, in_topic, self._on_odom, qos)

        self._count = 0
        self.get_logger().info(
            'Gazebo odom shim ready: %s -> %s + %s, rotating by %.4f rad '
            '(%.2f deg)%s'
            % (in_topic,
               self.get_parameter('odom_topic').value,
               self.get_parameter('zed_odom_topic').value,
               self._yaw, math.degrees(self._yaw),
               ', publishing %s -> %s' % (self._odom_frame, self._base_frame)
               if self._publish_tf else ', TF disabled'))
        if abs(self._yaw) < 1e-9:
            self.get_logger().warn(
                'spawn_yaw_rad is 0, so this node is a pass-through. That is '
                'correct only if the robot really was spawned facing world +x. '
                'The IGVC course spawns it at 2.356 rad; if you are seeing '
                'this on the IGVC course, the launch file is not passing the '
                'spawn yaw and the ground-truth grid will be 135 deg out.')

    def _on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        out.pose.pose.position.x = self._cos * px - self._sin * py
        out.pose.pose.position.y = self._sin * px + self._cos * py
        out.pose.pose.position.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        qx, qy, qz, qw = _quat_mul(self._rot, (q.x, q.y, q.z, q.w))
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # Body-frame twist: unchanged on purpose. See the module docstring.
        out.twist = msg.twist
        out.pose.covariance = msg.pose.covariance

        self._odom_pub.publish(out)
        self._zed_pub.publish(out)

        if self._tf is not None:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self._tf.sendTransform(tf)

        self._count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboOdomShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
