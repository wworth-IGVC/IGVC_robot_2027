"""
gazebo_nav_test.launch.py

The Gazebo twin of isaac_nav_test.launch.py: the simulator plus the
ground-truth navigation inputs, with no vision pipeline.

It starts gazebo_sim.launch.py and adds the nodes that turn a robot which can
be driven into a robot that drives itself:

  1. track_ground_truth_node   reads track_points.json and publishes the
                               perfect lane occupancy grid on
                               /lane_ground_truth
  2. gt_nav_bridge_node        stamps obstacle lethal cells into it and
                               republishes as /lane_map (Nav2 StaticLayer) and
                               /lane_costmap (the navigator's local crop)
  3. localization_node         owns map -> odom; in sim it is the identity
  4. Nav2                      controller_server, planner_server,
                               bt_navigator, velocity_smoother and
                               collision_monitor, from the SAME
                               nav2_lane_follow_config.yaml the real robot
                               uses
  5. igvc_navigator            extracts a lane path from /lane_costmap and
                               drives Nav2 through FollowPath, in local_lane
                               mode: no pre-known waypoints, no GPS

WHY THIS FILE IS THE PROOF, NOT JUST A CONVENIENCE

These are real 2026 nodes, unmodified, from igvc_lane_detection. They are
configured here with the SAME config file the Isaac nav test uses
(config/isaac_nav_test.yaml) and the SAME odometry remapping - every consumer
is remapped from /odom onto /front_zed_camera_x/zed_node/odom, the ZED contract
topic. If the RQ-03 rename in config/gazebo_bridge.yaml were wrong, these nodes
would sit silent, so this launch file is a test of the rename and not only a
user of it.

track_ground_truth_node in particular reads track_points.json and subscribes to
nothing at all, which makes it simulator-independent by construction. It was
the cheapest thing in the project that could be made to work against Gazebo and
it needed no shim whatsoever. gt_nav_bridge_node is the one that matters,
because it consumes simulator odometry: /lane_costmap only appears once BOTH
the ground-truth grid and odometry have arrived, and its crop follows the
robot, so it fails loudly when the odometry frame is wrong.

HOW THE VELOCITY COMMAND REACHES THE ROBOT, AND WHAT THAT SIDESTEPS

The real robot ends the Nav2 chain at /diff_drive_controller/cmd_vel_unstamped,
where ros2_control's diff_drive_controller turns a body twist into wheel
velocities and IsaacDriveHardware ships them to the simulator as
/isaac_joint_cmd. Gazebo's built-in DiffDrive takes the body twist directly on
/cmd_vel and does that conversion itself, so config/gazebo_nav_test_nav2_
overrides.yaml repoints collision_monitor's output and the loop closes with no
new code.

That is Stage 1 and it is deliberately NOT an answer to RQ-06. Everything
upstream - controller_server, the RegulatedPurePursuit gains, velocity_smoother
and its acceleration limits, collision_monitor and its stop polygons - is the
real robot's configuration untouched, so what is being exercised is the real
navigation stack. What is NOT exercised is ros2_control itself: joint-level
limits, the controller update rate and its interaction with the physics step.
Whether to close that last gap with gz_ros2_control in-process or with a
GazeboDriveHardware that mirrors IsaacDriveHardware over topics is RQ-06, it
has real control-loop-timing consequences, and it is a team decision rather
than something to settle in a launch file. /isaac_joint_cmd is still unbridged.

This file is already further than the Isaac nav test ever got:
isaac_nav_test.launch.py has every one of these same nodes commented out at
the bottom of its LaunchDescription, and section 3.8 of the research document
warns we have no evidence that pipeline ever fully worked.

Usage
-----
    ros2 launch igvc_test_bringup gazebo_nav_test.launch.py

    # with both windows, from a WSL2 shell
    ros2 launch igvc_test_bringup gazebo_nav_test.launch.py rviz:=true

Arguments
---------
    Everything gazebo_sim.launch.py takes, plus:
    track_image_file              path to track.png
    track_image_pixels_per_meter  float          (default: 52.5, JSON wins)
    debug_png_path                GT grid PNG export
    odom_topic                    odometry every consumer is remapped onto
                                  (default: /front_zed_camera_x/zed_node/odom)
    obstacle_source               where the planner's barrels come from
                                  (default: gt, the behaviour before 2026-10-01)
        gt         gt_nav_bridge_node stamps track_points.json's obstacles_m
                   into /lane_map and /lane_costmap
        none       it stamps none: the barrels are still physically in the
                   world but nothing tells the planner about them, so the
                   robot should hit any in its path (a negative control)
        nav2_scan  as none, plus Nav2's obstacle_layer on /scan alone, merged
                   from config/gazebo_nav_test_nav2_obstacle_scan.yaml
        lidar_map  as none, plus the competition's lidar_obstacle_costmap_node
                   (67a6934) on /scan, publishing /lidar_obstacle_map in odom,
                   which the competition navigator overlays onto the lane
                   region it searches (navigator_config.yaml
                   obstacle_map_topic). The node's parameters and the lidar
                   mount are the ones igvc_fused_drive.launch.py uses, the
                   mount read from the URDF by that file's own function.
        gt_map     the barrels from track_points.json, and only the barrels,
                   on /lidar_obstacle_map (gt_obstacle_map_node), the topic the
                   navigator overlays; the bridge's grid carries none. Valid
                   with any lane_source. The same cells as gt, by a different
                   route (scripts/gazebo/measurements/gt_obstacle_map_check.py)
    lane_source                   where the lane grids come from
                                  (default: gt, the behaviour before 2026-10-09)
        gt         gt_nav_bridge_node publishes /lane_map and /lane_costmap
        hough      the competition's lane_detection_node (OpenCV Hough, no
                   torch, runs in igvc_gazebo) does, from the front camera
        yolopv2    the competition's lane_segmentation_node does; it needs
                   torch and weights, so run it in igvc_dev_jazzy
        With hough or yolopv2, obstacle_source:=gt is promoted to gt_map (and
        the launch says so), because gt means "barrels stamped into the
        ground-truth lane grid", which is not the planner's grid then. The
        ground-truth bridge keeps running for scoring, on /gt/lane_map and
        /gt/lane_costmap; track_ground_truth_node's /lane_ground_truth is
        unchanged. Detector and navigator are started later than Nav2: see
        lane_warmup.
    lane_warmup                   seconds from the detector to the navigator in
                                  camera modes (default: auto, hough 5,
                                  yolopv2 45; a number overrides). Without it
                                  an empty grid reads as open road and the
                                  robot drives straight off.
    yolopv2_weights               weights path (default: $YOLOPV2_WEIGHTS)
    gt_outputs                    camera modes only: whether the ground-truth
                                  nodes run at all (default: true; false is
                                  the leak ablation, nothing of the ground
                                  truth is then in the graph)
"""

import json
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_root() -> str:
    env = os.environ.get("IGVC_WORKSPACE_ROOT", "")
    if env and os.path.isdir(env):
        return os.path.abspath(env)
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


# ── lane_source: where /lane_map and /lane_costmap come from ─────────────────
LANE_SOURCES = ("gt", "hough", "yolopv2")
# Seconds from the detector to the navigator in camera modes. Initial guesses,
# to be replaced by what test W5 measures (the navigator must not move the robot
# before there is lane evidence on /lane_costmap).
LANE_WARMUP_DEFAULT = {"gt": 0.0, "hough": 5.0, "yolopv2": 45.0}

# ── obstacle_source: where the planner's barrels come from ───────────────────
OBSTACLE_SOURCES = ("gt", "gt_map", "none", "nav2_scan", "lidar_map")


def _track_without_obstacles(track_file: str) -> str:
    """A copy of track_file with obstacles_m emptied, for gt_nav_bridge_node.

    Only the planner's grid loses the barrels. The world is the prebuilt
    igvc_course.sdf, and gazebo_sim.launch.py reads nothing but the spawn pose
    from track_file, so the barrels stay physically on the course: something
    else must now perceive them, or the robot drives into them. Half of them
    are in its path: a robot on the centreline would strike barrels 0, 2, 4
    and 5 (measured from track_points.json, 2026-10-01).
    """
    with open(track_file, encoding="utf-8") as fh:
        data = json.load(fh)
    data["obstacles_m"] = []
    out = os.path.join(tempfile.gettempdir(), "igvc_track_no_obstacles.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(data, fh)
    return out


def _shared_inflate(shared_cfg: str) -> float:
    """obstacle_inflate_radius_m exactly as gt_nav_bridge_node gets it.

    gt_obstacle_map_node must grow each barrel by the same margin the bridge
    does, or the two routes to the navigator would mark different cells. The
    value is read out of the shared YAML the bridge is launched with, so they
    cannot drift apart; 0.3 is the bridge's own code default, used when the key
    is absent, because that is what the bridge would then use.
    """
    with open(shared_cfg, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    params = (data.get("gt_nav_bridge_node") or {}).get("ros__parameters") or {}
    return float(params.get("obstacle_inflate_radius_m", 0.3))


def _merge(base: dict, over: dict) -> dict:
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def _merged_overrides(base_file: str, extra_file: str) -> str:
    """base_file with extra_file merged over it, written to a temp file.

    navigation_no_docking.launch.py takes one extra_params_file, so the sim's
    fixed overrides and an optional obstacle layer are kept in two files and
    joined here, rather than copied into a third that could drift.
    """
    with open(base_file, encoding="utf-8") as fh:
        base = yaml.safe_load(fh)
    with open(extra_file, encoding="utf-8") as fh:
        extra = yaml.safe_load(fh)
    out = os.path.join(tempfile.gettempdir(), "igvc_nav2_overrides_merged.yaml")
    with open(out, "w", encoding="utf-8") as fh:
        yaml.safe_dump(_merge(base, extra), fh, default_flow_style=None)
    return out


def _branch_lidar_mount() -> tuple:
    """The top lidar's (x, y, yaw) the competition computes from the URDF.

    igvc_fused_drive.launch.py reads test_robot.urdf.xacro for the rplidar_c1
    macro's origin and hands it to lidar_obstacle_costmap_node. This calls that
    file's own function rather than copying it, so the simulator cannot drift
    from the way the competition stack gets its mount. Importing the launch
    file only defines functions; generate_launch_description is not called.
    """
    import importlib.util

    path = _find(
        os.path.join("src", "igvc_test_bringup", "launch",
                     "igvc_fused_drive.launch.py"),
        "igvc_test_bringup",
        os.path.join("launch", "igvc_fused_drive.launch.py"))
    spec = importlib.util.spec_from_file_location("igvc_fused_drive_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._top_lidar_mount_from_urdf()


def _setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = cfg("use_sim_time").lower() in ("true", "1", "yes")
    odom_topic = cfg("odom_topic")
    lane_source = cfg("lane_source")
    if lane_source not in LANE_SOURCES:
        raise RuntimeError("lane_source:=%s is not one of %s"
                           % (lane_source, ", ".join(LANE_SOURCES)))
    obstacle_source = cfg("obstacle_source")
    if obstacle_source not in OBSTACLE_SOURCES:
        raise RuntimeError("obstacle_source:=%s is not one of %s"
                           % (obstacle_source, ", ".join(OBSTACLE_SOURCES)))
    # obstacle_source gt means "the barrels are stamped into the ground-truth
    # lane grid". With a camera detector that grid is not the planner's, so the
    # barrels would silently vanish. Promote to gt_map (the same barrels, on
    # the navigator's overlay topic) and say so. This must happen before
    # bridge_track is derived from obstacle_source.
    promoted = lane_source != "gt" and obstacle_source == "gt"
    if promoted:
        obstacle_source = "gt_map"
        print("NOTE: lane_source:=%s with obstacle_source:=gt. gt stamps the "
              "barrels into the ground-truth lane grid, which the planner does "
              "not read in this mode, so obstacle_source is promoted to gt_map "
              "(the same barrels on /lidar_obstacle_map)." % lane_source)
    bridge_track = cfg("track_file")
    if obstacle_source != "gt":
        bridge_track = _track_without_obstacles(bridge_track)

    warm_arg = cfg("lane_warmup")
    if warm_arg == "auto":
        lane_warmup = LANE_WARMUP_DEFAULT[lane_source]
    else:
        try:
            lane_warmup = float(warm_arg)
        except ValueError:
            raise RuntimeError("lane_warmup:=%s is not 'auto' or a number of "
                               "seconds" % warm_arg) from None
    gt_outputs = cfg("gt_outputs").lower() in ("true", "1", "yes")

    # YOLOPv2 needs torch and weights. Both are checked here, before anything is
    # built, so a missing one fails in the first second and not after the
    # navigator's start timer. Only igvc-dev-jazzy has torch (the simulator's
    # igvc_gazebo image deliberately does not).
    yolopv2_weights = ""
    if lane_source == "yolopv2":
        import importlib.util
        if importlib.util.find_spec("torch") is None:
            raise RuntimeError(
                "lane_source:=yolopv2 needs torch, which only igvc-dev-jazzy "
                "has. Run this in igvc_dev_jazzy (igvc_dev_jazzy_linux on "
                "native Linux), not igvc_gazebo.")
        yolopv2_weights = (cfg("yolopv2_weights")
                           or os.environ.get("YOLOPV2_WEIGHTS", ""))
        if not yolopv2_weights or not os.path.isfile(yolopv2_weights):
            raise RuntimeError(
                "YOLOPv2 weights not found (%r). Run "
                "src/igvc_lane_detection/scripts/fetch_yolopv2_weights.sh, "
                "set YOLOPV2_WEIGHTS, or pass yolopv2_weights:=" % yolopv2_weights)
    if lane_source != "gt" and cfg("sim_cameras") == "none":
        print("WARNING: lane_source:=%s with sim_cameras:=none. No image will "
              "arrive. This is the negative control, not a working setup."
              % lane_source)

    # The same config the Isaac nav test uses. Nothing in it is Isaac-specific:
    # it is grid geometry, obstacle inflation and navigator gains. Sharing it
    # is deliberate, so the two simulators cannot drift apart in the one place
    # where a difference would be invisible.
    shared_cfg = _find(
        os.path.join("src", "igvc_test_bringup", "config",
                     "isaac_nav_test.yaml"),
        "igvc_test_bringup",
        os.path.join("config", "isaac_nav_test.yaml"),
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_find(
            os.path.join("src", "igvc_test_bringup", "launch",
                         "gazebo_sim.launch.py"),
            "igvc_test_bringup",
            os.path.join("launch", "gazebo_sim.launch.py"),
        )),
        launch_arguments={
            "world": cfg("world"),
            "track_file": cfg("track_file"),
            "robot": cfg("robot"),
            "sim_cameras": cfg("sim_cameras"),
            # Forwarded so the camera budget can be varied on the AUTONOMY
            # path, not only on gazebo_sim.launch.py. Until 2026-09-17 these
            # three were neither declared here nor passed through, which locked
            # every autonomy and perception run to 640x360 at 15 Hz and made
            # "does YOLOPv2 tolerate 640x360" unmeasurable on the one launch
            # file that runs the full stack.
            "sim_camera_width": cfg("sim_camera_width"),
            "sim_camera_height": cfg("sim_camera_height"),
            "sim_camera_hz": cfg("sim_camera_hz"),
            "headless": cfg("headless"),
            "rviz": cfg("rviz"),
            "use_sim_time": cfg("use_sim_time"),
        }.items(),
    )

    # ── the free win: reads JSON, subscribes to nothing ──────────────────────
    gt_node = Node(
        package="igvc_lane_detection",
        executable="track_ground_truth_node",
        name="track_ground_truth_node",
        output="screen",
        parameters=[
            shared_cfg,
            {
                "use_sim_time": use_sim_time,
                "track_file": cfg("track_file"),
                "track_image_file": cfg("track_image_file"),
                "track_image_pixels_per_meter": float(
                    cfg("track_image_pixels_per_meter")),
                "debug_png_path": cfg("debug_png_path"),
            },
        ],
    )

    # ── the one that consumes simulator odometry ─────────────────────────────
    #
    # The remapping is the point. gt_nav_bridge_node subscribes to /odom in its
    # source; remapping it onto the ZED contract topic is exactly what
    # isaac_nav_test.launch.py does, so if gazebo_bridge.yaml got the rename
    # wrong this node never publishes /lane_costmap.
    #
    # It reads the odometry pose as a pose in the ground-truth grid's frame,
    # with no TF lookup anywhere, which is why gazebo_odom_shim has to correct
    # the frame at the source rather than leaving it to map -> odom.
    #
    # With a camera detector (lane_source hough or yolopv2) /lane_map and
    # /lane_costmap belong to the detector, which hard-codes the same two
    # names, and exactly one node may publish them. The bridge is moved onto
    # /gt/... so it can still be scored against; only names that collide move.
    gt_remaps = [("/odom", odom_topic)]
    if lane_source != "gt":
        gt_remaps += [("/lane_map", "/gt/lane_map"),
                      ("/lane_costmap", "/gt/lane_costmap")]
    gt_bridge_node = Node(
        package="igvc_lane_detection",
        executable="gt_nav_bridge_node",
        name="gt_nav_bridge_node",
        output="screen",
        remappings=gt_remaps,
        parameters=[
            shared_cfg,
            {
                "use_sim_time": use_sim_time,
                # The only node whose track file depends on obstacle_source:
                # it is the one that stamps barrels into the planner's grid.
                "track_file": bridge_track,
            },
        ],
    )

    # ── map -> odom ──────────────────────────────────────────────────────────
    #
    # Identity is CORRECT here, but only because gazebo_odom_shim has already
    # rotated the odometry into world axes with its origin at the spawn point,
    # which is the same frame track_ground_truth_node builds its grid in.
    # Against raw Gazebo odometry identity would be wrong by the spawn yaw.
    #
    # gps_enabled: False is what selects sim mode and seeds the identity. The
    # Isaac launch files also pass force_identity_map_to_odom here, and it is
    # NOT copied: six launch files set that parameter and no node anywhere in
    # the repo declares or reads it, so it has never done anything. Passing it
    # is harmless - an undeclared override is ignored - but propagating it
    # would suggest it is the thing switching the behaviour, and it is not.
    localization_node = Node(
        package="igvc_lane_detection",
        executable="localization_node",
        name="igvc_localization",
        output="screen",
        remappings=[("/odom", odom_topic)],
        parameters=[{
            "use_sim_time": use_sim_time,
            "gps_enabled": False,
            "max_odom_age_sec": 0.5,
        }],
    )

    # The ground-truth nodes run in gt mode (they ARE the lane source) and, by
    # default, in camera modes (scoring only). gt_outputs:=false leaves them out
    # of a camera run altogether, which is the ablation that shows nothing in
    # the graph was reading them.
    actions = [sim]
    if lane_source == "gt" or gt_outputs:
        actions += [gt_node, gt_bridge_node]
    actions.append(localization_node)

    # ── obstacles from the lidar, the way the 2026 robot got them ────────────
    #
    # Parameters are igvc_fused_drive.launch.py:246-275, unchanged except that
    # the odometry topic is this launch's. The grid is in odom, which in the
    # simulator is world-aligned with its origin at the spawn point
    # (gazebo_odom_shim), and the course lies well inside its 100 x 100 m.
    lidar_mount = None
    if obstacle_source == "lidar_map":
        lidar_mount = _branch_lidar_mount()
        actions.append(Node(
            package="igvc_lane_detection",
            executable="lidar_obstacle_costmap_node",
            name="lidar_obstacle_costmap_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "frame_id": "odom",
                "scan_topic": "/scan",
                "output_topic": "/lidar_obstacle_map",
                "use_odom_pose": True,
                "odom_topic": odom_topic,
                "max_odom_age_sec": 2.0,
                "lidar_x_m": lidar_mount[0],
                "lidar_y_m": lidar_mount[1],
                "lidar_yaw_rad": lidar_mount[2],
                "width_m": 100.0,
                "height_m": 100.0,
                "origin_x": -50.0,
                "origin_y": -50.0,
                "resolution": 0.10,
                "min_range_m": float(cfg("lidar_min_range_m")),
                "max_range_m": 10.0,
                "hit_weight": 5.0,
                "free_weight": 0.5,
                "hit_threshold": 4.0,
                "free_threshold": 3.0,
                "inflate_radius_m": float(cfg("lidar_inflate_radius_m")),
                "publish_hz": 5.0,
            }],
        ))

    # ── the barrels alone, on the navigator's overlay topic ──────────────────
    #
    # obstacle_source gt_map: the barrels the ground-truth bridge would have
    # stamped into /lane_costmap, published instead on /lidar_obstacle_map (the
    # topic navigator_config.yaml names), so a camera detector can own the lane
    # grid and the barrels still reach the planner. Never started with
    # lidar_map: both would publish that topic, and obstacle_source is one value.
    #
    # track_file is the ORIGINAL, with its obstacles_m: the bridge's copy has
    # them emptied. Inflation is read from the same YAML the bridge gets, and the
    # grid is 80 x 80 m like /lane_ground_truth, which holds the whole course.
    if obstacle_source == "gt_map":
        actions.append(Node(
            package="igvc_test_bringup",
            executable="gt_obstacle_map_node",
            name="gt_obstacle_map_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "track_file": cfg("track_file"),
                "output_topic": "/lidar_obstacle_map",
                "frame_id": "odom",
                "width_m": 80.0,
                "height_m": 80.0,
                "origin_x": -40.0,
                "origin_y": -40.0,
                "resolution": 0.10,
                "inflate_radius_m": _shared_inflate(shared_cfg),
                "publish_hz": 1.0,
            }],
        ))

    # ── lanes from the camera ────────────────────────────────────────────────
    #
    # The competition's own detector, its own protected config loaded verbatim,
    # nothing tuned. The node name must equal the YAML's top-level key or the
    # params file does not apply. It starts in its own timer, at the Nav2 delay,
    # so TF (map -> odom, odom -> base_link, the camera frames) is up first: a
    # detector that starts before it falls back to a pinhole projection and puts
    # the lanes in the wrong place (the 2026-09-29 Hough run). Inline values are
    # only what the simulator needs: sim time, one camera, the odometry topic.
    detector = None
    if lane_source == "hough":
        detector = Node(
            package="igvc_lane_detection",
            executable="lane_detection_node",
            name="lane_detection_node",
            output="screen",
            parameters=[
                _find(
                    os.path.join("src", "igvc_test_bringup", "config",
                                 "lane_detection_config.yaml"),
                    "igvc_test_bringup",
                    os.path.join("config", "lane_detection_config.yaml")),
                {
                    "use_sim_time": use_sim_time,
                    # The config says 3 (the real robot's cameras); the
                    # simulator has the front one, sim_cameras:=front.
                    "num_cameras": 1,
                },
            ],
        )
    elif lane_source == "yolopv2":
        detector = Node(
            package="igvc_lane_detection",
            executable="lane_segmentation_node",
            name="lane_segmentation_node",
            output="screen",
            parameters=[
                _find(
                    os.path.join("src", "igvc_test_bringup", "config",
                                 "lane_segmentation_config.yaml"),
                    "igvc_test_bringup",
                    os.path.join("config", "lane_segmentation_config.yaml")),
                {
                    "use_sim_time": use_sim_time,
                    "model_weights": yolopv2_weights,
                    # A parameter, so a remapping would do nothing.
                    "odom_topic": odom_topic,
                },
            ],
        )
    if detector is not None:
        actions.append(TimerAction(period=float(cfg("nav2_delay")),
                                   actions=[detector]))

    # ── Nav2 and the navigator: the part that makes the robot drive itself ───
    use_nav2 = cfg("nav2").lower() in ("true", "1", "yes")
    nav2_overrides = _find(
        os.path.join("src", "igvc_test_bringup", "config",
                     "gazebo_nav_test_nav2_overrides.yaml"),
        "igvc_test_bringup",
        os.path.join("config", "gazebo_nav_test_nav2_overrides.yaml"))
    if obstacle_source == "nav2_scan":
        nav2_overrides = _merged_overrides(nav2_overrides, _find(
            os.path.join("src", "igvc_test_bringup", "config",
                         "gazebo_nav_test_nav2_obstacle_scan.yaml"),
            "igvc_test_bringup",
            os.path.join("config", "gazebo_nav_test_nav2_obstacle_scan.yaml")))
    if use_nav2:
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_find(
                os.path.join("src", "igvc_test_bringup", "launch",
                             "navigation_no_docking.launch.py"),
                "igvc_test_bringup",
                os.path.join("launch", "navigation_no_docking.launch.py"),
            )),
            launch_arguments={
                "params_file": _find(
                    os.path.join("src", "igvc_test_bringup", "config",
                                 "nav2_lane_follow_config.yaml"),
                    "igvc_test_bringup",
                    os.path.join("config", "nav2_lane_follow_config.yaml")),
                "extra_params_file": nav2_overrides,
                "use_sim_time": cfg("use_sim_time"),
            }.items(),
        )

        # The navigator extracts a local lane path from /lane_costmap and
        # drives Nav2 through the FollowPath action. nav_strategy local_lane
        # means it uses only the costmap, with no pre-known centreline
        # waypoints and no GPS - the robot works out where the lane goes.
        #
        # Its odom_topic default is already /front_zed_camera_x/zed_node/odom,
        # the section 3.7 contract name, so it needs no Gazebo-specific
        # override to find the simulator's odometry. That is the RQ-03 rename
        # paying for itself.
        navigator_node = Node(
            package="igvc_lane_detection",
            executable="navigation_node",
            name="igvc_navigator",
            output="screen",
            remappings=[("/odom", odom_topic)],
            parameters=[
                shared_cfg,
                _find(
                    os.path.join("src", "igvc_test_bringup", "config",
                                 "navigator_config.yaml"),
                    "igvc_test_bringup",
                    os.path.join("config", "navigator_config.yaml")),
                {
                    "use_sim_time": use_sim_time,
                    "gps_enabled": False,
                    "follow_path_enabled": True,
                    "nav_strategy": "local_lane",
                    "odom_topic": odom_topic,
                    # navigator_config.yaml is shared with the real-robot
                    # flows and has strict freshness gates. /lane_costmap is
                    # published at 5 Hz here, so relax them in this last
                    # override layer exactly as isaac_nav_test.launch.py does.
                    "max_costmap_age_sec": 5.0,
                    "max_odom_age_sec": 0.5,
                    "max_odom_costmap_skew_sec": 5.0,
                    # The competition navigator (67a6934) rejects any path that
                    # turns more than max_path_heading_rad (default 0.75, 43
                    # deg) from the robot and then holds and reverses. The
                    # simulated course turns right by about 45 deg within 3 m of
                    # the start line (lateral offset -2.97 m at 3.2 m ahead,
                    # from track_points.json), and its limiter clamps to exactly
                    # the limit while its validator rejects anything over it, so
                    # the robot never moves (autonomy_check: 0.0 m in 400 s,
                    # measured 2026-10-08). The limit is raised here, in the sim
                    # only, to the most the validator accepts (pi/2), which is
                    # what main did before the merge: no limit. The clamp and
                    # validator inconsistency is a competition-code defect,
                    # recorded in docs/GAZEBO_SETUP.md and not fixed here.
                    # nav_max_path_heading_rad changes it; the default is the
                    # value above. With the validator tolerance in navigator.py
                    # the real robot's 0.75 works too (P1b).
                    "max_path_heading_rad": float(cfg("nav_max_path_heading_rad")),
                },
            ],
        )
        # ── why Nav2 starts late ─────────────────────────────────────────────
        #
        # Nav2 brings ten lifecycle nodes up in sequence, each with a bond the
        # lifecycle manager waits on, and the whole sequence aborts if any one
        # of them is slow: "Failed to change state for node: smoother_server",
        # after which nothing navigates. Starting that sequence at the same
        # instant as Gazebo means it competes with mesh loading and the first
        # GPU render, and on this laptop controller_server took 3.4 s of a 4 s
        # bond timeout before the next node failed outright.
        #
        # The lifecycle manager's bond_timeout cannot be raised from here -
        # navigation_no_docking.launch.py constructs that node with only
        # autostart and node_names, and does not pass it the params file - so
        # the fix is to stop the race instead of widening the window.
        #
        # It also spares the navigator its first few seconds of complaining
        # about a costmap and an odometry topic that do not exist yet.
        #
        # In camera modes the navigator waits lane_warmup longer than Nav2. An
        # empty grid reads as open road (the navigator treats unknown as
        # drivable), so it would drive straight off before the detector had
        # drawn a line, and with no grid at all it logs "no /lane_costmap",
        # holds, and reverses. The warm-up is a guess; test W5 measures it.
        delay = float(cfg("nav2_delay"))
        if lane_source == "gt":
            actions.append(TimerAction(period=delay,
                                       actions=[nav2, navigator_node]))
        else:
            actions.append(TimerAction(period=delay, actions=[nav2]))
            actions.append(TimerAction(period=delay + lane_warmup,
                                       actions=[navigator_node]))

    print("gazebo_nav_test.launch.py")
    print("  shared config : %s" % shared_cfg)
    print("  odom topic    : %s" % odom_topic)
    print("  nav2          : %s" % ("on" if use_nav2 else "off"))
    # /tmp/autonomy.log is the only record of which configuration a run used.
    print("  lanes         : %s (/lane_costmap and /lane_map from %s)"
          % (lane_source,
             {"gt": "gt_nav_bridge_node", "hough": "lane_detection_node",
              "yolopv2": "lane_segmentation_node"}[lane_source]))
    if lane_source == "yolopv2":
        print("  weights       : %s" % yolopv2_weights)
    if lane_source == "gt":
        print("  ground truth  : feeds the planner (gt_outputs does not apply)")
    elif gt_outputs:
        print("  ground truth  : /gt/lane_map, /gt/lane_costmap, scoring only")
    else:
        print("  ground truth  : not started (gt_outputs:=false)")
    print("  obstacles     : %s%s (planner track file %s; nav2 overrides %s)"
          % (obstacle_source, " (promoted from gt)" if promoted else "",
             bridge_track, nav2_overrides))
    if obstacle_source == "gt_map":
        print("  barrel overlay: gt_obstacle_map_node on /lidar_obstacle_map")
    if lane_source != "gt" and use_nav2:
        print("  navigator     : starts at nav2_delay + %.1f s (lane_warmup %s)"
              % (lane_warmup, cfg("lane_warmup")))
    if lidar_mount is not None:
        print("  lidar mount   : x %.3f y %.3f yaw %.4f (from the URDF, as "
              "igvc_fused_drive.launch.py reads it)" % lidar_mount)

    return actions


def generate_launch_description():
    repo = _repo_root()
    default_world = _find(
        os.path.join("src", "igvc_test_description", "worlds",
                     "igvc_course.sdf"),
        "igvc_test_description",
        os.path.join("worlds", "igvc_course.sdf"),
    )
    default_track = os.path.join(repo, "IGVC_track_generator",
                                 "track_points.json")
    default_track_image = os.path.join(repo, "IGVC_track_generator",
                                       "track.png")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("track_file", default_value=default_track),
        DeclareLaunchArgument("track_image_file",
                              default_value=default_track_image),
        DeclareLaunchArgument("track_image_pixels_per_meter",
                              default_value="52.5"),
        DeclareLaunchArgument("debug_png_path",
                              default_value="/tmp/igvc_nav_ground_truth.png"),
        DeclareLaunchArgument("robot", default_value="2027"),
        DeclareLaunchArgument("sim_cameras", default_value="front"),
        # Same defaults as gazebo_sim.launch.py, restated here only so they can
        # be overridden on the autonomy path. Raising these costs real time:
        # three RGBD cameras at 1280x720/30 delivered 7 Hz each on the RTX 5070
        # Ti Laptop, against 11 Hz each at 640x360/15. Count messages, never
        # real_time_factor, which sits near 1.0 while the cameras starve.
        DeclareLaunchArgument("sim_camera_width", default_value="640"),
        DeclareLaunchArgument("sim_camera_height", default_value="360"),
        DeclareLaunchArgument("sim_camera_hz", default_value="15"),
        DeclareLaunchArgument(
            "nav2", default_value="true",
            description="Start Nav2 and the IGVC navigator, so the robot "
                        "drives the course itself. nav2:=false gives the "
                        "ground-truth grids only, which is the right setting "
                        "for debugging the grids without a moving robot."),
        DeclareLaunchArgument(
            "nav2_delay", default_value="15.0",
            description="Seconds to wait before starting Nav2, so its ten "
                        "lifecycle nodes are not activating while Gazebo is "
                        "still loading meshes and rendering its first frame."),
        DeclareLaunchArgument(
            "lane_source", default_value="gt",
            description="Where /lane_map and /lane_costmap come from: gt "
                        "(gt_nav_bridge_node, the default), hough (the "
                        "competition's lane_detection_node, from the front "
                        "camera) or yolopv2 (lane_segmentation_node; needs "
                        "torch and weights, run it in igvc_dev_jazzy). With "
                        "hough or yolopv2, obstacle_source gt is promoted to "
                        "gt_map."),
        DeclareLaunchArgument(
            "lane_warmup", default_value="auto",
            description="lane_source hough or yolopv2: seconds from the "
                        "detector to the navigator. auto = hough 5, yolopv2 "
                        "45; a number overrides. The detector starts with "
                        "Nav2, at nav2_delay."),
        DeclareLaunchArgument(
            "yolopv2_weights", default_value="",
            description="lane_source:=yolopv2: path to the weights. Empty "
                        "means $YOLOPV2_WEIGHTS."),
        DeclareLaunchArgument(
            "gt_outputs", default_value="true",
            description="lane_source hough or yolopv2: whether the "
                        "ground-truth nodes run at all (scoring only, on "
                        "/gt/...). false is the leak ablation."),
        DeclareLaunchArgument(
            "obstacle_source", default_value="gt",
            description="Where the planner's barrels come from: gt (stamped "
                        "from track_points.json, the default), gt_map (the "
                        "same barrels, alone, on /lidar_obstacle_map), none "
                        "(not at all; a negative control), nav2_scan (Nav2's "
                        "obstacle_layer on /scan alone), lidar_map (the "
                        "competition's lidar_obstacle_costmap_node). The "
                        "barrels stay in the world in every case."),
        DeclareLaunchArgument(
            "lidar_inflate_radius_m", default_value="0.20",
            description="obstacle_source:=lidar_map only: the lidar map's "
                        "inflation, default the competition's 0.20 m "
                        "(igvc_fused_drive.launch.py). A diagnosis knob."),
        DeclareLaunchArgument(
            "lidar_min_range_m", default_value="0.15",
            description="obstacle_source:=lidar_map only: the lidar map's "
                        "minimum range, default the competition's 0.15 m. "
                        "A diagnosis knob."),
        DeclareLaunchArgument(
            "nav_max_path_heading_rad", default_value="1.57",
            description="The navigator's max_path_heading_rad in the simulator. "
                        "Default 1.57, the value the simulator has used since "
                        "2026-10-08 (the competition's own default is 0.75). "
                        "A diagnosis knob."),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/front_zed_camera_x/zed_node/odom",
            description="Odometry topic every consumer is remapped onto. The "
                        "default is the section 3.7 contract name, which is "
                        "what isaac_nav_test.launch.py uses."),
        OpaqueFunction(function=_setup),
    ])
