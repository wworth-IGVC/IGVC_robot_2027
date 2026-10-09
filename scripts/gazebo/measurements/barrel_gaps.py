"""How much room does each barrel leave inside the painted lane?

    python3 barrel_gaps.py [<inflate_m> ...]

For every barrel: the lane half-width at its nearest centreline vertex (inner
edge of the paint, generate_igvc_world.track_inner_edge_m), the barrel's lateral
offset from the centreline, its radius, and the free corridor on each side of
it once the barrel is grown by each given inflation radius (default 0.20 0.30
0.40). A side is passable for the chassis when its corridor is at least the
chassis width (0.810 m); the lane gate uses the rotated footprint, so a
corridor a few cm over that is still marginal. Run inside the container so the
generator imports.
"""
import json
import math
import sys

R = '/root/ros2_ws/src/IGVC_robot_2026'
sys.path.insert(0, R + '/scripts/gazebo')
from generate_igvc_world import track_inner_edge_m  # noqa: E402

CHASSIS_W = 0.810
t = json.load(open(R + '/IGVC_track_generator/track_points.json', encoding='utf-8'))
C = [(p['x'], p['y']) for p in t['centerline_m']]
B = [(o['x_m'], o['y_m'], o['radius_m']) for o in t['obstacles_m']]
n = len(C)
infl = [float(a) for a in sys.argv[1:]] or [0.20, 0.30, 0.40]


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
    return best, bi, (1.0 if side > 0 else -1.0)


print('barrel  centreline_idx  half_width  offset(+left)  radius   corridor left/right (m) per inflation')
for k, (bx, by, r) in enumerate(B):
    d, i, s = nearest(bx, by)
    hw = track_inner_edge_m(i / n)
    off = s * d
    row = []
    for a in infl:
        left = hw - (off + r + a)
        right = (off - r - a) + hw
        row.append('%.2f: %+.2f/%+.2f' % (a, left, right))
    print('%d  %4d  %.2f  %+.2f  %.2f   %s' % (k, i, hw, off, r, '   '.join(row)))
print('chassis width %.3f m; a corridor below that is not passable' % CHASSIS_W)
