# RQ-03 findings, from a 9-agent parallel audit

**Produced 2026-09-15** by five parallel code readers over the downstream stack,
one design synthesis, and three adversarial critics. 1.1M tokens, 336 tool
calls, zero agent errors.

**This is an audit, not an implementation.** It is the input to RQ-03, produced
before and alongside the work itself. Nothing in the repo was changed to produce
it. Where it and the implementation disagree, the implementation is newer -
check `git log` on the files it names.

Its value is the list of things that fail **silently**: a wrong TF frame that
still draws plausible lane lines, a camera_info topic that never arrives and
logs nothing, a synchroniser that never fires because a parameter still says
three cameras. Those are the ones worth re-testing after any change here.

**All three critics returned FLAWED** on the first design. They agreed on what
was right and independently found the same blocker. Read items 1 and 2 before
writing any code.

---

## 1. BLOCKER: the Gazebo container cannot run any downstream node

`docker/Dockerfile.gazebo-jazzy` installs exactly: `ros-jazzy-ros-gz`,
`gz-ros2-control`, `simulation-interfaces`, `xacro`, `robot-state-publisher`,
`rviz2`, `teleop-twist-keyboard`, `mesa-utils`, `vulkan-tools`,
`python3-colcon-common-extensions`.

**Verified by grep: it has no `cv_bridge`, no `python3-opencv`, no
`image_geometry`, no `message_filters`, no Nav2.**

`lane_detection_node` needs all four of the first group. `start_sim.sh` also
builds only three packages, so `igvc_lane_detection` is not even importable.

**Nothing downstream can run until the image is rebuilt.** Plan for that up
front rather than discovering it after the naming work is done. Two critics
called this the single reason the original design could not execute.

## 2. A real bug, independently confirmed, and it predates RQ-03

```text
src/zed-description/urdf/zed_macro.urdf.xacro:236
    <link name="${name}_left_camera_frame_optical"/>

src/igvc_test_description/urdf/gazebo/gazebo_sim.urdf.xacro:171
    <gz_frame_id>${prefix}_zed_camera_x_left_camera_optical_frame</gz_frame_id>
```

`_frame_optical` versus `_optical_frame`. **The simulated camera stamps every
image with a frame_id that does not exist in the TF tree.** One-word fix in the
xacro.

**Why it matters more than it looks.** `lane_detection.py:298-302` falls back to
**pinhole-only projection** when the TF lookup fails. It still publishes
`/lane_costmap`, RViz still draws an overlay with plausible lane lines, and the
geometry is silently wrong by however far the camera sits from `base_link`. The
only tell is a log line throttled to every 2 seconds:

```text
No TF from <frame> to base_link; falling back to pinhole projection for cam[0]
```

**Any test must assert the ABSENCE of that line**, not merely the presence of
`/lane_costmap`. This is the same class of failure as the llvmpipe fallback.

## 3. Things assumed and never verified

- **`camera_info` topic name.** The bridge assumes gz derives it as
  `<topic>/camera_info`. `GAZEBO_SETUP.md` records the bridge verified for
  Image, LaserScan and Clock only - **CameraInfo is not on that list**. If the
  name is wrong, `lane_detection` never gets intrinsics, `self.K[idx]` stays
  `None`, and the node runs with no error and produces nothing. Check
  `gz topic -l | grep front_zed` inside the container before trusting it. The
  node logs `Camera[0] intrinsics received.` exactly once
  (`lane_detection.py:271`); its absence is the tell.
- **Point cloud orientation.** The SDF tags all four rgbd outputs with one
  `gz_frame_id`, the optical z-forward frame. Images and depth are 2D rasters
  so that is right; the cloud may be generated x-forward, in which case tagging
  it optical rotates it 90 degrees. Nav2's `obstacle_layer` would then mark the
  floor as lethal overhead and the robot refuses to move for no visible reason.
  Defer the cloud until `/scan`-based navigation already works, so a failure has
  an obvious cause.

## 4. Renames that a launch remapping cannot reach

- **`/odom` is not remappable.** `navigator.py:164` and
  `lane_segmentation.py:127` read the odom topic from a **ROS parameter**, so a
  launch remap is a silent no-op. Needs a params override setting
  `odom_topic: "/odom"`.
- **`num_cameras` defaults to 3** (`lane_detection_config.yaml:4`) while the sim
  runs one camera by default. The `ApproximateTimeSynchronizer` for cam1 and
  cam2 never fires and **there is no error**. Override to 1.
- **Nav2 publishes velocity to `/diff_drive_controller/cmd_vel_unstamped`**
  (`nav2_lane_follow_config.yaml:133`), not `/cmd_vel`. Gazebo's DiffDrive
  listens on `/cmd_vel`. Needs a remap, and keeping `/cmd_vel` as the default
  preserves keyboard teleop.

## 5. Corrections to things previously believed, including by me

- **`/lane_ground_truth` may NOT be simulator-independent.** I previously
  advised treating `track_ground_truth_node` as a free win. A critic disputes
  this with evidence: `track_ground_truth_node.py:245-260` offsets the track by
  `-robot_start_pose` so the start becomes (0,0) in odom, and lines 301-305
  carry an explicit Isaac Sim frame assumption. **Verify before relying on it.**
- **`odom_tf_bridge` is already commented out** in the Isaac launch files, so
  the claimed collision risk with Gazebo's own odom to base_link TF is not the
  live hazard it was described as. Still: the simulator should own that edge.
- **`/isaac_joint_state` and `/isaac_joint_cmd` are out of scope for RQ-03.**
  Their only consumer is `IsaacDriveHardware`, which is not launched in the
  Gazebo stack. That is RQ-06. The publisher and subscriber live in
  `src/grr_hardware/src/isaac_drive.cpp:17` and `:20`, inside
  `IsaacDriveHardware::on_init` - not in `base_hardware_interface.cpp`.
- **14 keys in `lane_detection_config.yaml` are never declared by
  `lane_detection.py`** and therefore silently do nothing, including
  `use_depth`, `lane_fusion_frame`, `required_tf_target_frame`,
  `occupancy_grid_topic` and `publish_overlay_debug`. Do not assume a config key
  is wired up.

## 6. Two design conclusions that survived all three critics

**Use bridge remappings plus one small params override. Do not write a shim
node.** The three usual arguments for a shim all fail on inspection:

- *Depth semantics differ* - they do not. `lane_detection.py:286` does
  `imgmsg_to_cv2(depth_msg, '32FC1') * depth_scale`; gz `rgbd_camera` depth is
  already 32FC1 in metres, same as ZED with `openni_depth_mode: false`.
- *Image encoding differs* - it does (gz `rgb8` vs ZED `bgra8`) and it does not
  matter, because `lane_detection.py:281` and `lane_segmentation.py:696` both
  request `'bgr8'` and cv_bridge converts. Nothing uses `passthrough`.
- *frame_id needs rewriting* - no, `<gz_frame_id>` already states the frame and
  the correct value is the one-word fix in item 2. A shim would be papering over
  a typo.

`topic_tools relay` was rejected too: an extra process per topic, a second
serialise hop on 640x360 RGB and 32FC1 depth at 15 Hz, and a second set of
timestamps that widens the synchroniser's 0.1 s slop budget.

**`igvc_simulation_interface` is the wrong home** for any Gazebo shim. Its whole
dependency surface is `simulation_interfaces` services that have no Gazebo
server on Harmonic (RQ-05), and it carries a `DEVENV_HOST_PATH` container to
host translation that exists only because Isaac ran outside the container.
Gazebo runs inside it.

## 7. One thing the rename will break

`scripts/gazebo/bringup_smoke_test.sh:86` asserts
`check_topic /front_zed/image sensor_msgs/msg/Image`. Renaming to contract names
breaks the one verified test in the repo. Update the test in the same change,
and keep it able to fail.

---

## The order the critics converged on

1. **Falsify the frame claim first, zero edits, about five minutes.** Run the
   sim as it exists, then `ros2 topic echo ... --field header.frame_id` and
   `ros2 run tf2_ros tf2_echo base_link front_zed_camera_x_left_camera_frame_optical`.
   One of those must fail. If neither fails, the premise of item 2 is wrong and
   everything downstream of it needs rethinking.
2. **Settle the `camera_info` name** with `gz topic -l` in the same session.
3. **Then** the rename, the params override, and the image rebuild.

Nothing above was applied to the repo. Another agent holds that tree.
