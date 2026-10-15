#!/usr/bin/env python3
"""Turn an Onshape URDF export of the 2027 robot into the sim's robot body.

    python3 scripts/gazebo/generate_robot_2027.py "<export.zip or unzipped dir>"

Onshape (Format: URDF, Geometry: STL, Binary, Medium) exports the assembly in
its own CAD frame with one link per part, joints from the mates, and masses
from the materials. This script rewrites that into what the rest of the sim
expects, so nothing downstream has to know which robot is loaded:

  * ROS axes: x forward, y left, z up. In this CAD the front is -Y.
  * The frame names the 2026 code, bridge and plugins use: base_link,
    chassis_link (identity to base_link), left/right_wheel_link and _joint,
    caster_raceway_link/_joint (swivel), caster_wheel_link/_joint (spin).
    The CAD links hang off chassis_link through a fixed joint, cad_root_joint.
  * base_link where the 2026 robot has it: 0.150 m ahead of the drive-axle
    midpoint (the chassis centre) and 0.2331 m above the ground, so the spawn
    height, LANE's footprint and ON THE SLAB need no change.
  * Both wheel joint axes +y (positive turns both forward), as in 2026.
  * Masses: the CAD has no materials yet (total about 0.03 kg), so the 2026
    values are used: 2.1118586 kg per drive wheel, 1.0005643 kg caster wheel,
    0.2597675 kg caster raceway, and a 20 kg chassis PLACEHOLDER with a box
    inertia from the 2027 footprint (the 2026 one was 0.001, not physical).
  * Collisions: none are exported, so primitives are added: a chassis box over
    the footprint, cylinders on the drive wheels, a low-friction sphere on the
    caster wheel.
  * Dropped: the ZED X cover plate and GMSL connector, which the exporter
    places 0.17 to 0.34 m from the camera (inside the linked "ZED X
    (official)" subassembly; the CAD itself is fine). Cosmetic only.

Writes, relative to the repo root:
  src/igvc_test_description/urdf/robot_2027/body_2027.urdf.xacro
  src/igvc_test_description/meshes/robot_2027/*.stl
and prints the measured geometry for the hand-written top level
(urdf/robot_2027/igvc_2027.urdf.xacro). Stdlib only. See docs/GAZEBO_SETUP.md
section 11.
"""
import math
import os
import shutil
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DESC = os.path.join(REPO, "src", "igvc_test_description")
OUT_XACRO = os.path.join(DESC, "urdf", "robot_2027", "body_2027.urdf.xacro")
OUT_MESHES = os.path.join(DESC, "meshes", "robot_2027")
MESH_URI = "package://igvc_test_description/meshes/robot_2027/"

BASE_LINK_HEIGHT = 0.2331      # above the ground, as the 2026 robot
BASE_AHEAD_OF_AXLE = 0.150     # chassis centre ahead of the drive axle, as 2026
CHASSIS_MASS = 20.0            # PLACEHOLDER until the CAD has materials
WHEEL_MASS = 2.1118586         # 2026 CAD values
CASTER_WHEEL_MASS = 1.0005643
CASTER_RACEWAY_MASS = 0.2597675
DROP_PREFIXES = ("cover_plate_wide___zed_x", "gmsl___fakra_connector")


# ---------------------------------------------------------------- transforms
def rpy_matrix(rpy):
    a, b, c = rpy
    ca, sa, cb, sb, cc, sc = (math.cos(a), math.sin(a), math.cos(b),
                              math.sin(b), math.cos(c), math.sin(c))
    return [[cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa],
            [sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa],
            [-sb, cb * sa, cb * ca]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def apply(t, p):
    return [sum(t[0][i][k] * p[k] for k in range(3)) + t[1][i]
            for i in range(3)]


def compose(a, b):
    return (mat_mul(a[0], b[0]), apply(a, b[1]))


def origin_of(elem):
    o = elem.find("origin")
    xyz = [float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()]
    return (rpy_matrix(rpy), xyz)


IDENTITY = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])


# ------------------------------------------------------------------ geometry
def stl_points(path, stride=3):
    data = open(path, "rb").read()
    n = struct.unpack("<I", data[80:84])[0]
    if 84 + 50 * n != len(data):
        raise SystemExit("not a binary STL: %s" % path)
    for i in range(0, n, stride):
        v = struct.unpack("<12f", data[84 + 50 * i:84 + 50 * i + 48])
        yield v[3:6]
        yield v[6:9]
        yield v[9:12]


def bounds(points):
    lo, hi = [1e9] * 3, [-1e9] * 3
    for p in points:
        for k in range(3):
            lo[k], hi[k] = min(lo[k], p[k]), max(hi[k], p[k])
    return lo, hi


def fmt(v):
    return " ".join("%.6g" % (x + 0.0) for x in v)


# ---------------------------------------------------------------------- main
def load_export(arg):
    if os.path.isfile(arg) and arg.lower().endswith(".zip"):
        tmp = tempfile.mkdtemp(prefix="robot2027_")
        with zipfile.ZipFile(arg) as z:
            for name in z.namelist():
                if name.startswith("/") or ".." in name:
                    raise SystemExit("unsafe path in zip: %s" % name)
            z.extractall(tmp)
        arg = tmp
    urdfs = [os.path.join(d, f) for d, _, fs in os.walk(arg) for f in fs
             if f.endswith(".urdf")]
    if len(urdfs) != 1:
        raise SystemExit("expected one .urdf in %s, found %d" % (arg, len(urdfs)))
    pkg = os.path.dirname(os.path.dirname(urdfs[0]))
    return urdfs[0], os.path.join(pkg, "meshes")


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    urdf_path, mesh_dir = load_export(sys.argv[1])
    robot = ET.parse(urdf_path).getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = robot.findall("joint")
    by_child = {j.find("child").get("link"): j for j in joints}

    def link_tf(name):
        if name not in by_child:
            return IDENTITY
        j = by_child[name]
        return compose(link_tf(j.find("parent").get("link")), origin_of(j))

    def link_points(name, frame_tf=None):
        tf = link_tf(name) if frame_tf is None else frame_tf
        for vis in links[name].findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            vt = compose(tf, origin_of(vis))
            for p in stl_points(os.path.join(mesh_dir, os.path.basename(mesh.get("filename")))):
                yield apply(vt, p)

    roots = set(links) - set(by_child)
    if len(roots) != 1:
        raise SystemExit("export is not one tree: roots %s" % roots)
    cad_root = roots.pop()
    moving = [j for j in joints if j.get("type") != "fixed"]
    if len(moving) != 4:
        raise SystemExit("expected 4 moving joints (2 wheels, caster spin and "
                         "swivel), found %d: %s"
                         % (len(moving), [j.get("name") for j in moving]))

    # identify the moving joints from their geometry, not their names
    info = []
    for j in moving:
        child = j.find("child").get("link")
        lo, hi = bounds(link_points(child))
        centre = [(lo[k] + hi[k]) / 2 for k in range(3)]
        size = [hi[k] - lo[k] for k in range(3)]
        tf = link_tf(child)
        axis_local = [float(v) for v in j.find("axis").get("xyz").split()]
        axis = [sum(tf[0][i][k] * axis_local[k] for k in range(3)) for i in range(3)]
        info.append(dict(joint=j, child=child, centre=centre, size=size,
                         axis=axis, axis_local=axis_local))
    swivel = [m for m in info if abs(m["axis"][2]) > 0.9]
    spinners = [m for m in info if abs(m["axis"][2]) < 0.2]
    if len(swivel) != 1 or len(spinners) != 3:
        raise SystemExit("cannot tell the caster swivel from the wheels")
    swivel = swivel[0]
    # the caster wheel is the spinner whose parent is the swivel's child
    caster = [m for m in spinners
              if m["joint"].find("parent").get("link") == swivel["child"]]
    if len(caster) != 1:
        raise SystemExit("caster wheel is not attached to the swivel")
    caster = caster[0]
    wheels = [m for m in spinners if m is not caster]
    radius = max(wheels[0]["size"]) / 2.0
    axle = [(wheels[0]["centre"][k] + wheels[1]["centre"][k]) / 2 for k in range(3)]
    # CAD forward is the direction from the axle to the caster, in the XY plane
    fwd = [caster["centre"][0] - axle[0], caster["centre"][1] - axle[1]]
    if abs(fwd[1]) < abs(fwd[0]) or fwd[1] > 0:
        raise SystemExit("expected the caster in front along CAD -Y; got %s" % fwd)
    ground = axle[2] - radius
    base_cad = [axle[0], axle[1] - BASE_AHEAD_OF_AXLE, ground + BASE_LINK_HEIGHT]
    # p_ros = Rz(+90) p_cad + t   (x_ros = -y_cad, y_ros = x_cad)
    rz = rpy_matrix([0, 0, math.pi / 2])
    t = [-v for v in apply((rz, [0, 0, 0]), base_cad)]
    cad_in_base = (rz, t)

    def to_base(p):
        return apply(cad_in_base, p)

    for w in wheels:
        w["centre_base"] = to_base(w["centre"])
    left = max(wheels, key=lambda m: m["centre_base"][1])
    right = min(wheels, key=lambda m: m["centre_base"][1])
    separation = left["centre_base"][1] - right["centre_base"][1]

    # ---- rewrite
    rename = {cad_root: "cad_root",
              left["child"]: "left_wheel_link", right["child"]: "right_wheel_link",
              swivel["child"]: "caster_raceway_link",
              caster["child"]: "caster_wheel_link"}
    jrename = {left["joint"].get("name"): "left_wheel_joint",
               right["joint"].get("name"): "right_wheel_joint",
               swivel["joint"].get("name"): "caster_raceway_joint",
               caster["joint"].get("name"): "caster_wheel_joint"}

    # drop the misplaced cosmetic camera parts, with their whole subtrees
    drop = {n for n in links if n.startswith(DROP_PREFIXES)}
    grew = True
    while grew:
        grew = False
        for j in joints:
            if j.find("parent").get("link") in drop and j.find("child").get("link") not in drop:
                drop.add(j.find("child").get("link"))
                grew = True

    out = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    macro = ET.SubElement(out, "xacro:macro", name="robot_body_2027")
    ET.SubElement(macro, "link", name="base_link")
    chassis = ET.SubElement(macro, "link", name="chassis_link")
    j = ET.SubElement(macro, "joint", name="rootJoint", type="fixed")
    ET.SubElement(j, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(j, "parent", link="base_link")
    ET.SubElement(j, "child", link="chassis_link")
    j = ET.SubElement(macro, "joint", name="cad_root_joint", type="fixed")
    ET.SubElement(j, "origin", xyz=fmt(t), rpy=fmt([0, 0, math.pi / 2]))
    ET.SubElement(j, "parent", link="chassis_link")
    ET.SubElement(j, "child", link="cad_root")

    # chassis box from every fixed part, in base_link axes
    moving_children = {m["child"] for m in info}

    def subtree(root_name):
        names, stack = set(), [root_name]
        while stack:
            n = stack.pop()
            names.add(n)
            stack += [jj.find("child").get("link") for jj in joints
                      if jj.find("parent").get("link") == n]
        return names
    moving_tree = set()
    for m in info:
        moving_tree |= subtree(m["child"])
    fixed_parts = [n for n in links if n not in moving_tree and n not in drop]
    pts = [to_base(p) for n in fixed_parts for p in link_points(n)]
    blo, bhi = bounds(pts)
    box_bottom = max(blo[2], -BASE_LINK_HEIGHT + 0.10)   # keep 10 cm clear of the ground
    box = [bhi[0] - blo[0], bhi[1] - blo[1], bhi[2] - box_bottom]
    box_c = [(bhi[0] + blo[0]) / 2, (bhi[1] + blo[1]) / 2, (bhi[2] + box_bottom) / 2]
    m = CHASSIS_MASS
    ixx = m / 12 * (box[1] ** 2 + box[2] ** 2)
    iyy = m / 12 * (box[0] ** 2 + box[2] ** 2)
    izz = m / 12 * (box[0] ** 2 + box[1] ** 2)
    inert = ET.SubElement(chassis, "inertial")
    ET.SubElement(inert, "origin", xyz=fmt(box_c), rpy="0 0 0")
    ET.SubElement(inert, "mass", value="%g" % m)
    ET.SubElement(inert, "inertia", ixx="%.4f" % ixx, ixy="0", ixz="0",
                  iyy="%.4f" % iyy, iyz="0", izz="%.4f" % izz)
    col = ET.SubElement(chassis, "collision")
    ET.SubElement(col, "origin", xyz=fmt(box_c), rpy="0 0 0")
    ET.SubElement(ET.SubElement(col, "geometry"), "box", size=fmt(box))

    os.makedirs(OUT_MESHES, exist_ok=True)
    for f in os.listdir(OUT_MESHES):
        os.remove(os.path.join(OUT_MESHES, f))
    used = set()
    for name, link in links.items():
        if name in drop:
            continue
        new = ET.SubElement(macro, "link", name=rename.get(name, name))
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            fname = os.path.basename(mesh.get("filename"))
            used.add(fname)
            v = ET.SubElement(new, "visual")
            o = vis.find("origin")
            if o is not None:
                ET.SubElement(v, "origin", xyz=o.get("xyz", "0 0 0"), rpy=o.get("rpy", "0 0 0"))
            ET.SubElement(ET.SubElement(v, "geometry"), "mesh", filename=MESH_URI + fname)
            mat = vis.find("material")
            if mat is not None:
                v.append(mat)
        newname = rename.get(name, name)
        if newname in ("left_wheel_link", "right_wheel_link", "caster_wheel_link", "caster_raceway_link"):
            lo, hi = bounds(link_points(name, IDENTITY))   # in the link's own frame
            c = [(lo[k] + hi[k]) / 2 for k in range(3)]
            s = sorted(hi[k] - lo[k] for k in range(3))
            mass = {"left_wheel_link": WHEEL_MASS, "right_wheel_link": WHEEL_MASS,
                    "caster_wheel_link": CASTER_WHEEL_MASS,
                    "caster_raceway_link": CASTER_RACEWAY_MASS}[newname]
            r = s[2] / 2.0
            # isotropic inertia: needs no orientation, and a wheel of this
            # size is close to isotropic anyway (0.5 m r^2 vs m(3r^2+h^2)/12)
            iso = max(0.5 * mass * r * r, mass * (3 * r * r + s[0] ** 2) / 12.0)
            inert = ET.SubElement(new, "inertial")
            ET.SubElement(inert, "origin", xyz=fmt(c), rpy="0 0 0")
            ET.SubElement(inert, "mass", value="%g" % mass)
            ET.SubElement(inert, "inertia", ixx="%.6f" % iso, ixy="0", ixz="0",
                          iyy="%.6f" % iso, iyz="0", izz="%.6f" % iso)
            if newname in ("left_wheel_link", "right_wheel_link"):
                # cylinder along the joint axis (local); its Z must map onto it
                ax = [float(v) for v in by_child[name].find("axis").get("xyz").split()]
                k = max(range(3), key=lambda i: abs(ax[i]))
                rpy = {0: [0, math.pi / 2, 0], 1: [math.pi / 2, 0, 0], 2: [0, 0, 0]}[k]
                col = ET.SubElement(new, "collision")
                ET.SubElement(col, "origin", xyz=fmt(c), rpy=fmt(rpy))
                ET.SubElement(ET.SubElement(col, "geometry"), "cylinder",
                              radius="%.4f" % r, length="%.4f" % s[0])
            elif newname == "caster_wheel_link":
                col = ET.SubElement(new, "collision")
                ET.SubElement(col, "origin", xyz=fmt(c), rpy="0 0 0")
                ET.SubElement(ET.SubElement(col, "geometry"), "sphere", radius="%.4f" % r)

    for jt in joints:
        child = jt.find("child").get("link")
        parent = jt.find("parent").get("link")
        if child in drop or parent in drop:
            continue
        new = ET.SubElement(macro, "joint", name=jrename.get(jt.get("name"), jt.get("name")),
                            type=jt.get("type"))
        o = jt.find("origin")
        if o is not None:
            ET.SubElement(new, "origin", xyz=o.get("xyz", "0 0 0"), rpy=o.get("rpy", "0 0 0"))
        ET.SubElement(new, "parent", link=rename.get(parent, parent))
        ET.SubElement(new, "child", link=rename.get(child, child))
        if jt.get("type") != "fixed":
            ax = [float(v) for v in jt.find("axis").get("xyz").split()]
            if child in (left["child"], right["child"]):
                tf = link_tf(child)
                world = [sum(tf[0][i][k] * ax[k] for k in range(3)) for i in range(3)]
                if apply((rz, [0, 0, 0]), world)[1] < 0:      # make +y in base_link
                    ax = [-v for v in ax]
            ET.SubElement(new, "axis", xyz=fmt(ax))

    for f in sorted(used):
        shutil.copy2(os.path.join(mesh_dir, f), os.path.join(OUT_MESHES, f))
    os.makedirs(os.path.dirname(OUT_XACRO), exist_ok=True)
    header = ("<?xml version=\"1.0\"?>\n<!-- GENERATED by scripts/gazebo/generate_robot_2027.py from an "
              "Onshape URDF export.\n     Do not edit by hand: re-export from Onshape and re-run the "
              "script. -->\n")
    ET.indent(out, space="  ")
    with open(OUT_XACRO, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(header + ET.tostring(out, encoding="unicode") + "\n")

    def show(v):
        return "(%s)" % ", ".join("%.4f" % x for x in v)
    print("wrote", os.path.relpath(OUT_XACRO, REPO), "and", len(used), "meshes")
    print("dropped (cosmetic, misplaced by the exporter):", sorted(drop))
    print("wheel radius %.4f m, separation %.4f m" % (radius, separation))
    print("left wheel centre", show(left["centre_base"]), " right", show(right["centre_base"]))
    print("caster wheel centre", show(to_base(caster["centre"])))
    print("chassis box", show(box), "centred", show(box_c))
    print("sim_wheel_separation=%.4f sim_wheel_radius=%.4f" % (separation, radius))


if __name__ == "__main__":
    main()
