#!/usr/bin/env python3
# Does the Gazebo world paint the same corridor Nav2 plans in? Ray-casts
# perpendicular to the centreline in track.png and compares against the lane
# boxes in the generated SDF. This is the check that caught the world and the
# grid disagreeing by a median of 0.83 m.
#   python3 world_vs_track_check.py <path to igvc_course.sdf>
# Run on the HOST (needs numpy and PIL); the REPO constant below is absolute.
import io, json, math, re, sys
import numpy as np
from PIL import Image

REPO = r"C:/IGVC 2027/IGVC_robot_2027"
SDF  = sys.argv[1]
d = json.load(io.open(REPO + "/IGVC_track_generator/track_points.json", encoding="utf-8"))
cen = [(p["x"], p["y"]) for p in d["centerline_m"]]
ppm = d["pixels_per_meter"]

# --- generated world: pull every lane box centre out of the SDF -------------
txt = io.open(SDF, encoding="utf-8").read()
blocks = re.findall(r'<model name="(lane_(?:left|right)_\d+)">.*?<pose>([^<]+)</pose>', txt, re.S)
pts = {"lane_left": [], "lane_right": []}
for name, pose in blocks:
    side = "lane_left" if name.startswith("lane_left") else "lane_right"
    v = pose.split()
    pts[side].append((float(v[0]), float(v[1])))
print("lane boxes parsed: left=%d right=%d" % (len(pts["lane_left"]), len(pts["lane_right"])))

# --- track.png: measure the corridor by perpendicular ray casting -----------
img = np.array(Image.open(REPO + "/IGVC_track_generator/track.png").convert("L"))
H, W = img.shape
cx_px, cy_px = W / 2.0, H / 2.0
def m_to_px(x, y):            # inverse of the JSON frame note: x left, y down
    return cx_px - x * ppm, cy_px + y * ppm

def ray(px, py, dx, dy):
    for s in np.arange(0.0, 400.0, 0.25):
        x, y = px + dx * s, py + dy * s
        ix, iy = int(round(x)), int(round(y))
        if not (0 <= ix < W and 0 <= iy < H):
            return None
        if img[iy, ix] > 200:
            return s / ppm
    return None

N = len(cen)
png_corr, gen_corr = [], []
LINE_W = 3.0 * 0.0254
for i in range(0, N, 5):
    ax, ay = cen[(i - 3) % N]; bx, by = cen[(i + 3) % N]
    tx, ty = bx - ax, by - ay
    mag = math.hypot(tx, ty)
    if mag < 1e-9: continue
    nx, ny = -ty / mag, tx / mag
    px, py = m_to_px(*cen[i])
    # perpendicular in pixel space
    dxl, dyl = -nx * ppm, ny * ppm
    dl = math.hypot(dxl, dyl); dxl, dyl = dxl/dl, dyl/dl
    a = ray(px, py, dxl, dyl); b = ray(px, py, -dxl, -dyl)
    if a is None or b is None: continue
    png_corr.append(a + b)
    # nearest generated lane box on each side -> inner edges
    cxm, cym = cen[i]
    dl_ = min(math.hypot(p[0]-cxm, p[1]-cym) for p in pts["lane_left"])
    dr_ = min(math.hypot(p[0]-cxm, p[1]-cym) for p in pts["lane_right"])
    gen_corr.append((dl_ - LINE_W/2) + (dr_ - LINE_W/2))

png_corr = np.array(png_corr); gen_corr = np.array(gen_corr)
err = gen_corr - png_corr
print("samples                 : %d" % len(png_corr))
print("track.png corridor  (m) : min %.3f  max %.3f  median %.3f" % (png_corr.min(), png_corr.max(), np.median(png_corr)))
print("generated corridor  (m) : min %.3f  max %.3f  median %.3f" % (gen_corr.min(), gen_corr.max(), np.median(gen_corr)))
print("error gen - png     (m) : median %+.3f  mean %+.3f  max|%.3f|" % (np.median(err), err.mean(), np.abs(err).max()))
print("within 0.15 m           : %.1f%%" % (100.0*np.mean(np.abs(err) < 0.15)))
