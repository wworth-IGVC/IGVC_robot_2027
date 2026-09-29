#!/usr/bin/env python3
"""Stand-ins for the five hardware drivers that cannot run without their devices.

Used by docs/COMPETITION_STACK.md, which runs the 2026 competition stack
(2026/more_diverging_changes at 67a6934) in a container with no robot attached.
Started by launch_stack.sh; you do not run it by hand.

Each stand-in has the real driver's node name and advertises topic names the
robot stack is configured to use. None of them publishes any data, except as
noted below, so `ros2 topic echo` on a camera, lidar or GPS topic prints
nothing. That is expected: there is no hardware and no simulator here.

  3 x zed_wrapper zed_node   /<camera>/zed_node, per sensor_launch.launch.py.
                             Topics are the ones this repo's configs name
                             (rgb, camera_info, depth, cloud, odom, imu). The
                             real node publishes more; not drawn.
  sllidar_node               /scan, per sllidar_c1_launch.py.
  ublox_gps_node             fix, fix_velocity only. The real driver publishes
                             more nav*/mon* topics (publish.nav.all: true).

One exception to "no data": the front camera publishes a stationary odom at
30 Hz, as a parked robot would. odom_tf_bridge turns it into odom->base_link,
without which Nav2's costmaps never activate.

Written 2026-09-24 to measure the robot's ROS graph
(local-notes/ros2_graph_robot_2026-09-24, off the repo); copied here unchanged
apart from this docstring on 2026-09-29.
"""
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix, PointCloud2

CAMERAS = ['left_zed_camera_x', 'front_zed_camera_x', 'right_zed_camera_x']
ZED_TOPICS = [
    (Image, 'rgb/color/rect/image'),
    (CameraInfo, 'rgb/color/rect/camera_info'),
    (Image, 'depth/depth_registered'),
    (CameraInfo, 'depth/camera_info'),
    (PointCloud2, 'point_cloud/cloud_registered'),
    (Odometry, 'odom'),
    (Imu, 'imu/data'),
]


def main():
    rclpy.init()
    nodes = []
    for cam in CAMERAS:
        n = Node('zed_node', namespace=cam)
        n.pubs = [n.create_publisher(t, '~/' + name, 10) for t, name in ZED_TOPICS]
        nodes.append(n)
        if cam == 'front_zed_camera_x':
            odom_pub = n.pubs[5]

            def tick(node=n, pub=odom_pub):
                msg = Odometry()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                msg.pose.pose.orientation.w = 1.0
                pub.publish(msg)
            n.create_timer(1.0 / 30.0, tick)
    lidar = Node('sllidar_node')
    lidar.pubs = [lidar.create_publisher(LaserScan, 'scan', 10)]
    gps = Node('ublox_gps_node')
    gps.pubs = [gps.create_publisher(NavSatFix, 'fix', 1),
                gps.create_publisher(TwistWithCovarianceStamped, 'fix_velocity', 1)]
    nodes += [lidar, gps]

    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
