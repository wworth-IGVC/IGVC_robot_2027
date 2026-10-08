"""Where did a run's chassis cross the paint, and which barrel was nearest?

    python3 lane_crossings.py <track_log> [<track_log> ...]

Same clearance arithmetic as autonomy_check.sh (rotated 0.810 x 0.970 m chassis
against the local painted half-width from generate_igvc_world.py), printed per
contiguous span of negative clearance: sim time, worst clearance, side of the
lane, world position, and the nearest barrel with its distance. Run inside the
container so the generator imports.
"""
import json
import math
import sys

R = '/root/ros2_ws/src/IGVC_robot_2026'
sys.path.insert(0, R + '/scripts/gazebo')
from generate_igvc_world import track_inner_edge_m  # noqa: E402

t = json.load(open(R + '/IGVC_track_generator/track_points.json', encoding='utf-8'))
C = [(p['x'], p['y']) for p in t['centerline_m']]
B = [(o['x_m'], o['y_m'], o['radius_m']) for o in t['obstacles_m']]
SP = (t['robot_start_pose']['position_m']['x'], t['robot_start_pose']['position_m']['y'])
n = len(C)


def nearest(x, y):
    best, bi, side = float('inf'), 0, 0.0
    for i in range(n):
        ax, ay = C[i]
        bx, by = C[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        px, py = ax + u * dx, ay + u * dy
        d = math.hypot(x - px, y - py)
        if d < best:
            best, bi = d, i
            side = dx * (y - py) - dy * (x - px)
    return best, bi, ('left' if side > 0 else 'right')


def tangent(i):
    ax, ay = C[(i - 2) % n]
    bx, by = C[(i + 2) % n]
    return math.atan2(by - ay, bx - ax)


for path in sys.argv[1:]:
    rows = []
    for line in open(path):
        try:
            v = [float(s) for s in line.split()]
        except ValueError:
            continue
        if len(v) >= 4:
            rows.append(v)
    spans, cur = [], None
    for x, y, ts, yaw in rows:
        # pose_logger.py writes world coordinates (odom plus the spawn point),
        # the same numbers autonomy_check.sh scores
        d, i, side = nearest(x, y)
        h = track_inner_edge_m(i / float(n))
        rel = math.atan2(math.sin(yaw - tangent(i)), math.cos(yaw - tangent(i)))
        reach = abs(0.405 * math.cos(rel)) + abs(0.485 * math.sin(rel))
        c = h - (d + reach)
        if c < 0:
            if cur is None:
                cur = {'t0': ts, 'worst': c, 'x': x, 'y': y, 'side': side}
            if c < cur['worst']:
                cur.update(worst=c, x=x, y=y, side=side)
            cur['t1'] = ts
        elif cur is not None:
            spans.append(cur)
            cur = None
    if cur is not None:
        spans.append(cur)
    print('== %s: %d span(s) over the paint' % (path, len(spans)))
    for s in spans:
        bd = min(((math.hypot(s['x'] - bx, s['y'] - by) - br, k)
                  for k, (bx, by, br) in enumerate(B)))
        print('   t %.1f to %.1f s, worst %+.3f m, %s side, at (%.1f, %.1f); '
              'nearest barrel_%d, %.2f m from its surface'
              % (s['t0'], s['t1'], s['worst'], s['side'], s['x'], s['y'], bd[1], bd[0]))
