"""
gazebo_sim.launch.py

Gazebo Harmonic bringup: start the IGVC course, spawn the robot, and bridge it
into ROS 2. This is the Gazebo counterpart to isaac_nav_test.launch.py, and it
is the file that turns "we have a simulator image" into "we have a robot that
drives".

Stage 1. The robot is driven by Gazebo's built-in DiffDrive system, declared in
urdf/gazebo/gazebo_sim.urdf.xacro. It does not exercise ros2_control, but it
does exercise the description, physics, sensors, odometry and TF in one go.
Stage 2 swaps DiffDrive for gz_ros2_control, which since the move to ROS 2
Jazzy ships as a binary and no longer needs a source build - see RQ-06 before
starting that.

The world and this launch file read the SAME track_points.json, so the course
geometry, the robot spawn pose and igvc_lane_detection's ground-truth navigator
all share one coordinate frame with no alignment step.

RUN IT FROM WSL2, NOT POWERSHELL, or there will be no window. See
docs/GAZEBO_SETUP.md section 3A.

Usage
-----
    # Once per container. Both packages are install-only, so this is seconds,
    # not a real build. It IS required: test_robot.urdf.xacro resolves its
    # includes with $(find igvc_test_description), which needs the ament index.
    cd /root/ros2_ws
    colcon build --symlink-install \
        --base-paths src/IGVC_robot_2026/src \
        --packages-select zed_description igvc_test_description igvc_test_bringup
    source install/setup.bash

    ros2 launch igvc_test_bringup gazebo_sim.launch.py

    # Then, from a second shell in the same container
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        "{linear: {x: 0.5}, angular: {z: 0.2}}"

Arguments
---------
    world              path to the world sdf      (default: generated IGVC course)
    track_file         path to track_points.json  (default: repo copy)
    sim_cameras        none | front | all         (default: front)
    sim_camera_width   int                        (default: 640)
    sim_camera_height  int                        (default: 360)
    sim_camera_hz      int                        (default: 15)
    headless           true | false               (default: false)
    use_sim_time       true | false               (default: true)
    robot_name         spawned entity name        (default: igvc_robot)
    spawn_{x,y,z,yaw}  override the json spawn pose

Camera budget, measured: three 1280x720 cameras at 30 Hz deliver about 7 Hz
each. The defaults here are the ones that hold up. Pass sim_cameras:=all
deliberately, and judge it by counting messages, not by real_time_factor -
gz-sim runs physics and rendering on separate threads, so RTF sits near 1.0
while cameras lag badly.
"""

import io
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ── Locating files in the install tree or the source tree ────────────────────
#
# The Isaac launch files do the same dance. Prefer the installed share copy so
# the file behaves normally once built, but fall back to the bind-mounted
# source tree, which is what you get when only some packages are installed.

def _repo_root() -> str:
    env = os.environ.get("IGVC_WORKSPACE_ROOT", "")
    if env and os.path.isdir(env):
        return os.path.abspath(env)
    # launch/ -> igvc_test_bringup/ -> src/ -> repo root. realpath first,
    # because --symlink-install makes __file__ a symlink into the install tree.
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def _find(rel_from_repo: str, pkg: str, rel_from_share: str) -> str:
    """Prefer the installed share copy; fall back to the source tree."""
    try:
        cand = os.path.join(get_package_share_directory(pkg), rel_from_share)
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    return os.path.join(_repo_root(), rel_from_repo)


def _setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    repo = _repo_root()
    world = cfg("world")
    track_file = cfg("track_file")
    robot_name = cfg("robot_name")
    headless = cfg("headless").lower() in ("true", "1", "yes")
    use_sim_time = cfg("use_sim_time").lower() in ("true", "1", "yes")
    cams = cfg("sim_cameras")

    # ── spawn pose: json unless explicitly overridden ────────────────────────
    sx, sy, syaw = -11.8816, 0.4516, 2.356194490192345
    if os.path.exists(track_file):
        try:
            with io.open(track_file, encoding="utf-8") as fh:
                start = json.load(fh)["robot_start_pose"]
            sx = float(start["position_m"]["x"])
            sy = float(start["position_m"]["y"])
            syaw = float(start["yaw_rad"])
        except Exception as exc:                      # noqa: BLE001
            print("gazebo_sim.launch.py: could not read spawn pose from "
                  "%s (%s); using the built-in default" % (track_file, exc))
    else:
        print("gazebo_sim.launch.py: %s not found; using the built-in "
              "default spawn pose" % track_file)

    for key, default in (("spawn_x", sx), ("spawn_y", sy), ("spawn_yaw", syaw)):
        val = cfg(key)
        if val != "":
            if key == "spawn_x":
                sx = float(val)
            elif key == "spawn_y":
                sy = float(val)
            else:
                syaw = float(val)
    sz = float(cfg("spawn_z"))

    xacro_file = _find(
        os.path.join("src", "igvc_test_description", "urdf", "robots",
                     "test_robot.urdf.xacro"),
        "igvc_test_description",
        os.path.join("urdf", "robots", "test_robot.urdf.xacro"),
    )

    robot_description = Command([
        "xacro ", xacro_file,
        " sim:=true",
        " sim_cameras:=", cfg("sim_cameras"),
        " sim_camera_width:=", cfg("sim_camera_width"),
        " sim_camera_height:=", cfg("sim_camera_height"),
        " sim_camera_hz:=", cfg("sim_camera_hz"),
    ])

    gz_args = world if headless else world + " -r"
    if headless:
        gz_args = world + " -r -s --headless-rendering"

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch",
            "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            # value_type=str is required. Without it launch tries to parse the
            # URDF as YAML and dies: the description is one long XML string,
            # not a structured parameter.
            "robot_description": ParameterValue(robot_description,
                                                value_type=str),
            "use_sim_time": use_sim_time,
        }],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_name,
            "-x", str(sx), "-y", str(sy), "-z", str(sz),
            "-Y", str(syaw),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── the bridge ───────────────────────────────────────────────────────────
    # Direction matters and is easy to get backwards:
    #   [  gz  -> ROS        ]  ROS -> gz        @  both
    # /cmd_vel is the only thing flowing INTO the simulator.
    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    ]

    # JointStatePublisher publishes gz.msgs.Model on a world-scoped topic, so
    # it has to be named in full and remapped back to the conventional name.
    world_name = "igvc_course"
    js_topic = "/world/%s/model/%s/joint_state" % (world_name, robot_name)
    bridge_args.append(js_topic + "@sensor_msgs/msg/JointState[gz.msgs.Model")

    cam_prefixes = []
    if cams in ("front", "all"):
        cam_prefixes.append("front")
    if cams == "all":
        cam_prefixes += ["left", "right"]
    for p in cam_prefixes:
        bridge_args += [
            "/%s_zed/image@sensor_msgs/msg/Image[gz.msgs.Image" % p,
            "/%s_zed/depth_image@sensor_msgs/msg/Image[gz.msgs.Image" % p,
            "/%s_zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" % p,
            "/%s_zed/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" % p,
        ]

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=bridge_args,
        remappings=[(js_topic, "/joint_states")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    print("gazebo_sim.launch.py")
    print("  repo root : %s" % repo)
    print("  world     : %s" % world)
    print("  spawn     : x=%.4f y=%.4f z=%.2f yaw=%.6f" % (sx, sy, sz, syaw))
    print("  cameras   : %s" % cams)
    print("  bridging  : %d topics" % len(bridge_args))

    return [gz, rsp, spawn, bridge]


def generate_launch_description():
    default_world = _find(
        os.path.join("src", "igvc_test_description", "worlds",
                     "igvc_course.sdf"),
        "igvc_test_description",
        os.path.join("worlds", "igvc_course.sdf"),
    )
    default_track = os.path.join(
        _repo_root(), "IGVC_track_generator", "track_points.json")

    # ── Gazebo mesh resolution, which is fiddlier than it looks ──────────────
    #
    # sdformat's URDF parser rewrites package:// into model://, so Gazebo looks
    # for model://zed_description/meshes/zedx.stl and needs a resource root
    # that CONTAINS a directory literally named zed_description.
    #
    # The source tree cannot provide that. The checkout is src/zed-description
    # with a HYPHEN, while the package is zed_description with an UNDERSCORE,
    # so model:// never resolves against src/. Only the install tree has the
    # directory under its real package name.
    #
    # So build the path from AMENT_PREFIX_PATH: every built package contributes
    # its <prefix>/share, which is exactly where model:// expects to look. The
    # src fallback stays for igvc_test_description, whose directory name does
    # happen to match, so partially-built workspaces still find the robot.
    share_roots = []
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        share = os.path.join(prefix, "share")
        if os.path.isdir(share) and share not in share_roots:
            share_roots.append(share)
    src_root = os.path.join(_repo_root(), "src")
    if os.path.isdir(src_root):
        share_roots.append(src_root)
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing:
        share_roots.append(existing)

    args = [
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("track_file", default_value=default_track),
        DeclareLaunchArgument("sim_cameras", default_value="front"),
        DeclareLaunchArgument("sim_camera_width", default_value="640"),
        DeclareLaunchArgument("sim_camera_height", default_value="360"),
        DeclareLaunchArgument("sim_camera_hz", default_value="15"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="igvc_robot"),
        DeclareLaunchArgument("spawn_x", default_value=""),
        DeclareLaunchArgument("spawn_y", default_value=""),
        DeclareLaunchArgument("spawn_z", default_value="0.25"),
        DeclareLaunchArgument("spawn_yaw", default_value=""),
    ]

    return LaunchDescription(
        [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(share_roots))]
        + args
        + [OpaqueFunction(function=_setup)]
    )
