"""
gt_obstacle_map.py

Publish the course's barrels, and only the barrels, as the competition
navigator's obstacle overlay, so lane perception and obstacle perception are
two independent switches in the simulator.

WHY THIS NODE EXISTS
--------------------
With lane_source:=gt, gt_nav_bridge_node stamps the barrels into /lane_costmap
together with the lane lines. With lane_source:=hough or yolopv2 that grid
comes from a camera detector and has no barrels in it. The navigator has a second,
independent obstacle input, /lidar_obstacle_map (navigator_config.yaml
obstacle_map_topic, navigator.py _overlay_obstacles): every cell >= 90 on it is
projected into the lane region the navigator searches and forced to 100. This
node fills that topic with the barrels from track_points.json, and nothing else.

It is the same topic lidar_obstacle_costmap_node fills on the real robot, so
the navigator cannot tell the two apart. The launch never starts both.

THE ARITHMETIC IS gt_nav_bridge_node's, ON PURPOSE
--------------------------------------------------
rasterise_barrels() reproduces GTNavBridgeNode._stamp_obstacles line for line:

    world  = (x - start.x, y - start.y)             no rotation
    radius = (r + inflate_radius_m) / resolution    in cells
    cell   = (world - origin) / resolution          float, not rounded
    stamp  = every cell whose index is within radius of that point

so a barrel covers the same cells here as it does in the ground-truth grid.
scripts/gazebo/measurements/gt_obstacle_map_check.py (test U1) compares the two
cell for cell, with a control that must differ.

One detail matters for that equality: the bridge reads resolution from a
message, where it is a float32. 0.1 becomes 0.10000000149011612. This module
rounds the resolution the same way before it does any arithmetic, so the cells
match what the bridge computes in the running graph, not what it would compute
from the Python literal.

Values: 100 inside a disc, -1 everywhere else. The grid says nothing about free
space. The navigator reads only cells >= 90 from this topic.

Parameters
    track_file        str     ''                    track_points.json (the ORIGINAL,
                                                    with its obstacles_m)
    output_topic      str     /lidar_obstacle_map   what navigator_config.yaml names
    frame_id          str     odom                  the frame the navigator assumes
    width_m           float   80.0
    height_m          float   80.0
    origin_x          float   -40.0
    origin_y          float   -40.0
    resolution        float   0.10
    inflate_radius_m  float   0.30                  the bridge's obstacle_inflate_radius_m
    publish_hz        float   1.0
    use_sim_time      bool    false
"""

from __future__ import annotations

import array
import json
import math
from pathlib import Path
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid


def load_barrels(track_file: str) -> Tuple[List[Tuple[float, float, float]],
                                           Tuple[float, float], List[str]]:
    """Barrels (x_m, y_m, radius_m) in raw track coordinates, the spawn point
    (x, y), and a list of problems.

    Parsed as GTNavBridgeNode._load_obstacles parses it: robot_start_pose
    .position_m for the origin offset, obstacles_m entries x_m, y_m, radius_m,
    a malformed entry skipped. A missing or unreadable file raises, because a
    node that quietly publishes an empty grid would make every barrel invisible
    and the robot would drive into them.
    """
    if not track_file:
        raise ValueError('track_file is empty')
    path = Path(track_file)
    if not path.exists():
        raise FileNotFoundError('track_file not found: %s' % track_file)
    payload = json.loads(path.read_text(encoding='utf-8'))

    pos = payload.get('robot_start_pose', {}).get('position_m', {})
    start = (float(pos.get('x', 0.0)), float(pos.get('y', 0.0)))

    barrels: List[Tuple[float, float, float]] = []
    problems: List[str] = []
    for obs in payload.get('obstacles_m', []):
        try:
            barrels.append((float(obs['x_m']), float(obs['y_m']),
                            float(obs['radius_m'])))
        except (KeyError, TypeError, ValueError) as exc:
            problems.append('skipping malformed obstacle entry: %r (%s)' % (obs, exc))
    return barrels, start, problems


def grid_size(width_m: float, height_m: float, resolution: float) -> Tuple[int, int]:
    """(columns, rows) of the grid, the way lidar_obstacle_costmap_node sizes it."""
    return (int(round(width_m / resolution)), int(round(height_m / resolution)))


def rasterise_barrels(barrels, start, width_cells: int, height_cells: int,
                      resolution: float, origin_x: float, origin_y: float,
                      inflate_radius_m: float) -> np.ndarray:
    """An int8 (rows, columns) array: 100 inside every barrel disc, -1 elsewhere.

    The body of the loop is GTNavBridgeNode._stamp_obstacles, unchanged except
    for the names. Do not "simplify" it: the point is that the two agree
    exactly. See the module docstring for the float32 resolution.
    """
    w = int(width_cells)
    h = int(height_cells)
    res = float(np.float32(resolution))
    ox = origin_x
    oy = origin_y
    data = np.full((h, w), -1, dtype=np.int8)
    if res <= 0.0:
        return data

    for obs_x_m, obs_y_m, obs_r_m in barrels:
        world_x = obs_x_m - start[0]
        world_y = obs_y_m - start[1]

        total_r_m = obs_r_m + inflate_radius_m
        total_r_cells = total_r_m / res

        cx = (world_x - ox) / res
        cy = (world_y - oy) / res

        imin = max(0, int(math.floor(cy - total_r_cells)))
        imax = min(h - 1, int(math.ceil(cy + total_r_cells)))
        jmin = max(0, int(math.floor(cx - total_r_cells)))
        jmax = min(w - 1, int(math.ceil(cx + total_r_cells)))

        if imax < imin or jmax < jmin:
            continue  # obstacle entirely outside grid

        js = np.arange(jmin, jmax + 1)
        is_ = np.arange(imin, imax + 1)
        jg, ig = np.meshgrid(js, is_)
        dist2 = (jg - cx) ** 2 + (ig - cy) ** 2
        in_circle = dist2 <= total_r_cells ** 2
        data[imin:imax + 1, jmin:jmax + 1][in_circle] = 100
    return data


class GTObstacleMapNode(Node):
    """Latch the barrels as an OccupancyGrid on the navigator's overlay topic."""

    def __init__(self) -> None:
        super().__init__('gt_obstacle_map_node')

        self.declare_parameter('track_file', '')
        self.declare_parameter('output_topic', '/lidar_obstacle_map')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('width_m', 80.0)
        self.declare_parameter('height_m', 80.0)
        self.declare_parameter('origin_x', -40.0)
        self.declare_parameter('origin_y', -40.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('inflate_radius_m', 0.30)
        self.declare_parameter('publish_hz', 1.0)

        track_file = str(self.get_parameter('track_file').value)
        self._topic = str(self.get_parameter('output_topic').value)
        self._frame = str(self.get_parameter('frame_id').value)
        width_m = float(self.get_parameter('width_m').value)
        height_m = float(self.get_parameter('height_m').value)
        self._origin_x = float(self.get_parameter('origin_x').value)
        self._origin_y = float(self.get_parameter('origin_y').value)
        self._res = float(self.get_parameter('resolution').value)
        inflate = float(self.get_parameter('inflate_radius_m').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self._nx, self._ny = grid_size(width_m, height_m, self._res)
        if self._nx <= 0 or self._ny <= 0 or self._res <= 0.0:
            raise ValueError('gt_obstacle_map: width/height/resolution must be > 0')

        barrels, start, problems = load_barrels(track_file)
        for problem in problems:
            self.get_logger().warn(problem)
        if not barrels:
            self.get_logger().error(
                'No obstacles_m in %s: the overlay will be empty and nothing '
                'will tell the navigator about the barrels.' % track_file)

        data = rasterise_barrels(barrels, start, self._nx, self._ny, self._res,
                                 self._origin_x, self._origin_y, inflate)
        self._data = array.array('b', data.tobytes())

        # Same QoS as lidar_obstacle_costmap_node and the navigator's
        # subscription: RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(OccupancyGrid, self._topic, map_qos)
        self.create_timer(1.0 / max(publish_hz, 0.1), self._publish)

        self.get_logger().info(
            f'gt_obstacle_map: {len(barrels)} barrel(s), '
            f'{int(np.count_nonzero(data == 100))} lethal cell(s), '
            f'{self._nx}x{self._ny} cells @ {self._res:.2f} m, '
            f'frame={self._frame}, out={self._topic} '
            f'(inflate +{inflate:.2f} m, start offset '
            f'({start[0]:.3f}, {start[1]:.3f}))')

    def _publish(self) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame
        grid.info.resolution = self._res
        grid.info.width = self._nx
        grid.info.height = self._ny
        grid.info.origin.position.x = self._origin_x
        grid.info.origin.position.y = self._origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = self._data
        self._pub.publish(grid)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GTObstacleMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
