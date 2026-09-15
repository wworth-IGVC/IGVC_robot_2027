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
"""

import os

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


def _setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = cfg("use_sim_time").lower() in ("true", "1", "yes")
    odom_topic = cfg("odom_topic")

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
            "sim_cameras": cfg("sim_cameras"),
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
    gt_bridge_node = Node(
        package="igvc_lane_detection",
        executable="gt_nav_bridge_node",
        name="gt_nav_bridge_node",
        output="screen",
        remappings=[("/odom", odom_topic)],
        parameters=[
            shared_cfg,
            {
                "use_sim_time": use_sim_time,
                "track_file": cfg("track_file"),
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

    actions = [sim, gt_node, gt_bridge_node, localization_node]

    # ── Nav2 and the navigator: the part that makes the robot drive itself ───
    use_nav2 = cfg("nav2").lower() in ("true", "1", "yes")
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
                "extra_params_file": _find(
                    os.path.join("src", "igvc_test_bringup", "config",
                                 "gazebo_nav_test_nav2_overrides.yaml"),
                    "igvc_test_bringup",
                    os.path.join("config",
                                 "gazebo_nav_test_nav2_overrides.yaml")),
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
        actions.append(TimerAction(period=float(cfg("nav2_delay")),
                                   actions=[nav2, navigator_node]))

    print("gazebo_nav_test.launch.py")
    print("  shared config : %s" % shared_cfg)
    print("  odom topic    : %s" % odom_topic)
    print("  nav2          : %s" % ("on" if use_nav2 else "off"))

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
        DeclareLaunchArgument("sim_cameras", default_value="front"),
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
