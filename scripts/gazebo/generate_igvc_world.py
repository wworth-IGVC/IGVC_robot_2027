#!/usr/bin/env python3
"""
Generate a Gazebo world for the IGVC course from IGVC_track_generator/track_points.json.

Why generate rather than convert the Isaac USD or the OpenSCAD track.stl:

  * track_points.json already carries centerline_m, obstacles_m and
    robot_start_pose in the SAME odom frame that igvc_lane_detection's
    navigator.py and gt_nav_bridge_node.py consume. Building the world from
    those numbers means the simulated course and the ground-truth navigator
    agree by construction, with no alignment step and no scale factor to get
    wrong.
  * track.png would work as a ground texture, but the JSON frame notes say
    "x left (negated pixel x), y down, origin at image center", so a texture
    needs two flips and an origin shift that are easy to get subtly wrong and
    hard to notice. The metre points need none.

Lane lines are emitted as thin white boxes offset perpendicular to the
centerline. They are visual-only and static: paint has no collision, so the
robot drives over them, and a camera still sees white lines on asphalt. The
barrels get both a visual and a collision, because the gpu_lidar renders
visuals while physics needs the collision.

Usage:
    python3 generate_igvc_world.py                 # writes the default output
    python3 generate_igvc_world.py --lane-width-ft 16 --out /tmp/wide.sdf

IGVC rules put the lane 10 to 20 ft wide, so --lane-width-ft is the knob most
worth turning. Default 12 ft.
"""

import argparse
import io
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

DEFAULT_JSON = os.path.join(REPO, "IGVC_track_generator", "track_points.json")
DEFAULT_OUT = os.path.join(
    REPO, "src", "igvc_test_description", "worlds", "igvc_course.sdf"
)

FT = 0.3048
LINE_WIDTH_M = 3.0 * 0.0254   # IGVC painted lines are about 3 inches
LINE_THICK_M = 0.005          # just proud of the ground so it never z-fights
BARREL_HEIGHT_M = 0.9


def resample(points, step_m):
    """Walk the polyline and emit a point every step_m along it."""
    out = [points[0]]
    acc = 0.0
    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg <= 1e-9:
            continue
        acc += seg
        if acc >= step_m:
            out.append((bx, by))
            acc = 0.0
    return out


def offset_polyline(points, dist, closed):
    """Offset a polyline sideways by dist, using the averaged vertex normal."""
    n = len(points)
    out = []
    for i in range(n):
        if closed:
            px, py = points[(i - 1) % n]
            nx_, ny_ = points[(i + 1) % n]
        else:
            px, py = points[max(i - 1, 0)]
            nx_, ny_ = points[min(i + 1, n - 1)]
        tx, ty = nx_ - px, ny_ - py
        mag = math.hypot(tx, ty)
        if mag <= 1e-9:
            continue
        # Left normal of the tangent.
        ox, oy = -ty / mag, tx / mag
        out.append((points[i][0] + ox * dist, points[i][1] + oy * dist))
    return out


def segment_boxes(points, closed, name_prefix, width, thickness, rgba, emissive):
    """Emit one static visual-only box per polyline segment.

    The emissive term is not decoration. RQ-08: the Isaac floor binds track.png
    to BOTH diffuseColor and emissiveColor, deliberately, so the painted lane
    lines stay visible regardless of scene lighting. That is called out as "an
    easy thing to lose in conversion", and losing it makes every lane-detection
    result depend on where the sun happens to be. Reproduced here as an
    emissive material term on the line geometry.
    """
    parts = []
    n = len(points)
    last = n if closed else n - 1
    for i in range(last):
        ax, ay = points[i]
        bx, by = points[(i + 1) % n]
        length = math.hypot(bx - ax, by - ay)
        if length <= 1e-6:
            continue
        cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0
        yaw = math.atan2(by - ay, bx - ax)
        # Overlap slightly so corners do not show gaps.
        length += width * 0.5
        parts.append(
            f"""    <model name="{name_prefix}_{i}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {thickness / 2.0:.4f} 0 0 {yaw:.5f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{length:.4f} {width:.4f} {thickness:.4f}</size></box></geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>{emissive}</emissive>
          </material>
        </visual>
      </link>
    </model>"""
        )
    return parts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--lane-width-ft", type=float, default=12.0,
                    help="IGVC rules allow 10 to 20 ft (default: 12)")
    ap.add_argument("--step-m", type=float, default=0.6,
                    help="lane segment length; smaller is smoother and slower")
    ap.add_argument("--lane-emissive", type=float, default=0.55,
                    help="RQ-08: emissive term on lane paint, 0 to 1 "
                         "(default: 0.55). 0 reproduces plain diffuse paint "
                         "and loses the Isaac behaviour")
    ap.add_argument("--barrel-collision", dest="barrel_collision",
                    action="store_true", default=True,
                    help="RQ-11, the default: barrels are solid")
    ap.add_argument("--no-barrel-collision", dest="barrel_collision",
                    action="store_false",
                    help="RQ-11: match Isaac's generated field.usd, where "
                         "obstacles are visual-only and the robot drives "
                         "straight through them")
    args = ap.parse_args()

    with io.open(args.json, encoding="utf-8") as fh:
        d = json.load(fh)

    centerline = [(p["x"], p["y"]) for p in d["centerline_m"]]
    obstacles = d.get("obstacles_m", [])
    start = d["robot_start_pose"]

    gap = math.hypot(centerline[0][0] - centerline[-1][0],
                     centerline[0][1] - centerline[-1][1])
    closed = gap < 1.0
    if closed:
        centerline = centerline[:-1]

    coarse = resample(centerline, args.step_m)
    half = (args.lane_width_ft * FT) / 2.0
    left = offset_polyline(coarse, half, closed)
    right = offset_polyline(coarse, -half, closed)

    xs = [p[0] for p in centerline]
    ys = [p[1] for p in centerline]
    pad = 6.0
    ground_x = (max(xs) - min(xs)) + 2 * pad
    ground_y = (max(ys) - min(ys)) + 2 * pad
    ground_cx = (max(xs) + min(xs)) / 2.0
    ground_cy = (max(ys) + min(ys)) / 2.0

    white = "0.95 0.95 0.95 1"
    # RQ-08. Not full white: emissive 1.0 would blow out the camera and make
    # the lines unusable for thresholding. This keeps them lit in shadow
    # without saturating. Tune with --lane-emissive and re-measure; RQ-08 also
    # asks for a measurable check rather than eyeballing it, which does not
    # exist yet.
    emissive = "%.2f %.2f %.2f 1" % ((args.lane_emissive,) * 3)

    models = []
    models += segment_boxes(left, closed, "lane_left", LINE_WIDTH_M,
                            LINE_THICK_M, white, emissive)
    models += segment_boxes(right, closed, "lane_right", LINE_WIDTH_M,
                            LINE_THICK_M, white, emissive)

    for i, o in enumerate(obstacles):
        r = o["radius_m"]
        # RQ-11 is OPEN and this line is the decision. Isaac's generated
        # field.usd gives obstacles no CollisionAPI, no RigidBodyAPI and no
        # mass, so there the robot drives straight through them; whether that
        # was deliberate or an oversight is unknown. Solid is the default here
        # because an obstacle course the robot cannot hit does not test
        # obstacle avoidance. Cylinder primitives rather than the STL meshes,
        # because RQ-11 notes Gazebo mesh collision is expensive and it costs
        # real-time factor directly. Pass --no-barrel-collision to match Isaac.
        collision = "" if not args.barrel_collision else f"""
        <collision name="collision">
          <geometry><cylinder><radius>{r:.4f}</radius><length>{BARREL_HEIGHT_M}</length></cylinder></geometry>
        </collision>"""
        models.append(
            f"""    <model name="barrel_{i}">
      <static>true</static>
      <pose>{o['x_m']:.4f} {o['y_m']:.4f} {BARREL_HEIGHT_M / 2.0:.4f} 0 0 0</pose>
      <link name="link">{collision}
        <visual name="visual">
          <geometry><cylinder><radius>{r:.4f}</radius><length>{BARREL_HEIGHT_M}</length></cylinder></geometry>
          <material>
            <ambient>0.9 0.35 0.05 1</ambient>
            <diffuse>0.9 0.35 0.05 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
        )

    header = f"""<?xml version="1.0" ?>
<!--
  GENERATED FILE - do not edit by hand.
  Regenerate with scripts/gazebo/generate_igvc_world.py

  Source      : IGVC_track_generator/track_points.json ({d.get('schema')})
  Frame       : {d.get('frame')}, identical to centerline_m, so this world and
                igvc_lane_detection's ground-truth navigator share coordinates.
  Centerline  : {len(centerline)} points, {'closed loop' if closed else 'open path'}
  Lane width  : {args.lane_width_ft:.1f} ft ({args.lane_width_ft * FT:.3f} m)
  Lane segs   : {len(left)} left + {len(right)} right at {args.step_m} m
  Lane paint  : emissive {args.lane_emissive:.2f} - RQ-08. Isaac binds track.png to
                BOTH diffuseColor and emissiveColor so lane lines stay visible
                regardless of scene lighting. Dropping it would make every
                lane-detection result depend on the sun angle.
  Barrels     : {len(obstacles)}, collision {'ON' if args.barrel_collision else 'OFF'} - RQ-11 is OPEN.
                Isaac's generated field.usd gives obstacles NO collision at all,
                so there the robot drives through them. Solid is the default
                here; the no-barrel-collision flag matches Isaac instead.
                (An XML comment cannot contain a double hyphen, so the flag is
                named without its leading dashes.)
  Robot start : x={start['position_m']['x']:.4f} y={start['position_m']['y']:.4f} yaw={start['yaw_rad']:.6f} rad ({start['yaw_deg']:.1f} deg)

  The launch file reads the spawn pose from the SAME json, so it is not
  duplicated here. See igvc_test_bringup/launch/gazebo_sim.launch.py.
-->
<sdf version="1.9">
  <world name="igvc_course">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.55 0.68 0.85 1</background>
      <grid>false</grid>
    </scene>

    <!-- Asphalt. Deliberately dark so the white lane paint has contrast for
         the camera; lane detection is the reason this world exists. -->
    <model name="ground">
      <static>true</static>
      <pose>{ground_cx:.4f} {ground_cy:.4f} -0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{ground_x:.2f} {ground_y:.2f} 0.1</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{ground_x:.2f} {ground_y:.2f} 0.1</size></box></geometry>
          <material>
            <ambient>0.18 0.18 0.19 1</ambient>
            <diffuse>0.22 0.22 0.23 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""

    footer = """
  </world>
</sdf>
"""

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with io.open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(header)
        fh.write("\n".join(models))
        fh.write(footer)

    print("wrote %s" % args.out)
    print("  lane segments : %d left, %d right" % (len(left), len(right)))
    print("  barrels       : %d" % len(obstacles))
    print("  ground        : %.1f x %.1f m centred (%.2f, %.2f)"
          % (ground_x, ground_y, ground_cx, ground_cy))
    print("  models total  : %d" % len(models))
    print("  size          : %.1f KB" % (os.path.getsize(args.out) / 1024.0))


if __name__ == "__main__":
    main()
