#!/usr/bin/env python3
"""Does the simulated 2D lidar see the barrels, in the frame Nav2 would use?

Question: could Nav2's stock ObstacleLayer, fed ONLY by /scan, mark the course
barrels correctly, with no dependence on the camera point cloud, which is
known to be rotated 90 degrees (P0-3)?

Method: robot parked (nav2 off), collect 20 scans, transform every finite
return through TF into `odom`, the frame the costmaps use, and compare with
the barrel positions predicted from track_points.json:
    barrel_odom = obstacles_m - spawn_position
gazebo_odom_shim publishes odom in world axes with its origin at the spawn.
Returns closer than 0.6 m are the robot's own structure, inside the footprint
the costmap clears, and are counted but not scored.

PASS needs >= 95% of the remaining returns within 0.20 m of a barrel surface
and at least one barrel hit. Two negative controls must FAIL or the test is
void: NEG-A skips the transform; NEG-B applies the spawn yaw twice. Use a
non-zero spawn yaw, or NEG-B equals the real transform and proves nothing.

From the default spawn every barrel is beyond the lidar's 12 m range, so park
the robot near one. Inside the container, with the workspace built:

    SPAWN_X=-2.851 SPAWN_Y=-9.201 SPAWN_YAW=0.7854       ros2 launch igvc_test_bringup gazebo_sim.launch.py headless:=true         spawn_x:=-2.851 spawn_y:=-9.201 spawn_yaw:=0.7854 &
    # wait until /odom publishes, then, with the same SPAWN_* exported:
    python3 scripts/gazebo/measurements/lidar_barrel_check.py

Result 2026-09-24, tier A, that pose: PASS. 100.0% of 480 returns beyond the
footprint explained, barrels at 2.77, 5.80 and 8.89 m all hit, NEG-A and
NEG-B 0.0%. 80 own-body returns at 0.10 to 0.13 m, bearing 161 to 164 deg.
Run it from a file, never an inline `bash -c` containing "ros2 launch":
sim_preflight's pkill -f matches its own caller's command line.
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
import tf2_ros

TRACK = "/root/ros2_ws/src/IGVC_robot_2026/IGVC_track_generator/track_points.json"
N_SCANS = 20
TOL = 0.20


class Probe(Node):
    def __init__(self):
        super().__init__("lidar_barrel_probe",
                         parameter_overrides=[rclpy.parameter.Parameter(
                             "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tfl = tf2_ros.TransformListener(self.buf, self)
        self.scans = []
        self.create_subscription(LaserScan, "/scan", self.cb, 10)

    def cb(self, m):
        if len(self.scans) < N_SCANS:
            self.scans.append(m)


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    d = json.load(open(TRACK))
    sx, sy = d["robot_start_pose"]["position_m"]["x"], d["robot_start_pose"]["position_m"]["y"]
    syaw = d["robot_start_pose"]["yaw_rad"]
    # A spawn override moves the odom origin with it (gazebo_odom_shim takes
    # the same overridden yaw), so predictions must use the override too.
    import os
    if os.environ.get("SPAWN_X"):
        sx, sy = float(os.environ["SPAWN_X"]), float(os.environ["SPAWN_Y"])
        syaw = float(os.environ["SPAWN_YAW"])
        print("spawn override  : x=%.3f y=%.3f yaw=%.3f" % (sx, sy, syaw))
    barrels = [(o["x_m"] - sx, o["y_m"] - sy, o["radius_m"]) for o in d["obstacles_m"]]

    rclpy.init()
    n = Probe()
    t_end = time.time() + 60
    while time.time() < t_end and len(n.scans) < N_SCANS:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.scans:
        print("RESULT: NO SCANS received on /scan in 60 s")
        return 2
    frame = n.scans[0].header.frame_id
    tf = None
    t_end = time.time() + 20
    while time.time() < t_end and tf is None:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            tf = n.buf.lookup_transform("odom", frame, Time())
        except Exception as e:
            last = e
    if tf is None:
        print("RESULT: TF odom <- %s unavailable: %s" % (frame, last))
        return 2
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    tz = tf.transform.translation.z
    yaw = quat_yaw(tf.transform.rotation)
    print("scan frame      : %s   (%d scans, %d beams each, range %.2f-%.1f m)" % (
        frame, len(n.scans), len(n.scans[0].ranges), n.scans[0].range_min, n.scans[0].range_max))
    print("TF odom<-lidar  : x=%.3f y=%.3f z=%.3f yaw=%.2f deg   (spawn yaw from JSON %.2f deg)" % (
        tx, ty, tz, math.degrees(yaw), math.degrees(syaw)))

    # lidar-frame points, all scans pooled
    pts = []
    own = 0
    for s in n.scans:
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min < r < 0.6:
                # Inside the robot's own footprint: its structure, which the
                # costmap clears (footprint_clearing_enabled). Counted, not scored.
                own += 1
                continue
            if math.isfinite(r) and s.range_min < r < s.range_max:
                a = s.angle_min + i * s.angle_increment
                pts.append((r * math.cos(a), r * math.sin(a)))
    print("own-body returns: %d (range < 0.6 m, inside the footprint; not scored)" % own)
    print("finite returns  : %d over %d scans (%.1f per scan), beyond the footprint" % (len(pts), len(n.scans), len(pts) / len(n.scans)))
    if not pts:
        print("RESULT: FAIL, the lidar returned nothing finite; barrels invisible to /scan")
        return 1

    def score(label, xf):
        hits = [0] * len(barrels)
        explained = 0
        for (px, py) in pts:
            wx, wy = xf(px, py)
            best, bi = 1e9, -1
            for j, (bx, by, br) in enumerate(barrels):
                dd = abs(math.hypot(wx - bx, wy - by) - br)
                if dd < best:
                    best, bi = dd, j
            if best <= TOL:
                explained += 1
                hits[bi] += 1
        frac = explained / len(pts)
        ok = frac >= 0.95 and any(hits)
        print("%-6s explained %5.1f%%  barrels hit %d/8  per-barrel returns %s  -> %s" % (
            label, 100 * frac, sum(1 for h in hits if h), hits, "PASS" if ok else "FAIL"))
        return ok, hits

    c, s_ = math.cos(yaw), math.sin(yaw)
    real = lambda px, py: (tx + c * px - s_ * py, ty + s_ * px + c * py)
    nega = lambda px, py: (px, py)
    c2, s2 = math.cos(yaw + syaw), math.sin(yaw + syaw)
    negb = lambda px, py: (tx + c2 * px - s2 * py, ty + s2 * px + c2 * py)

    ok, hits = score("REAL", real)
    # Where are the returns no barrel explains? Near the lidar means the
    # robot's own structure; the costmap clears inside the footprint.
    stray = []
    for (px, py) in pts:
        wx, wy = real(px, py)
        if min(abs(math.hypot(wx - bx, wy - by) - br) for bx, by, br in barrels) > TOL:
            stray.append((round(math.hypot(px, py), 2), round(math.degrees(math.atan2(py, px)))))
    uniq = sorted(set(stray))
    print("unexplained     : %d returns, %d distinct (range m, bearing deg in lidar frame): %s"
          % (len(stray), len(uniq), uniq[:12]))
    oka, _ = score("NEG-A", nega)
    okb, _ = score("NEG-B", negb)

    print("\nbarrel  odom_x  odom_y  dist_from_lidar  returns")
    for j, (bx, by, br) in enumerate(barrels):
        print("  %d    %6.2f  %6.2f     %5.2f m         %d" % (j, bx, by, math.hypot(bx - tx, by - ty), hits[j]))

    valid = (not oka) and (not okb)
    print("\nnegative controls fail as they must: %s" % ("YES" if valid else "NO, TEST INVALID"))
    print("RESULT: %s" % ("PASS" if ok and valid else "FAIL"))
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok and valid else 1


if __name__ == "__main__":
    sys.exit(main())
