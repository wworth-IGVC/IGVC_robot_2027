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
was right and independently found the same blocker - which then turned out to
be **four-fifths wrong**; see item 1, kept and corrected, for both the fact and
the method error that produced it. Item 2 is the finding that held up and is
the one worth acting on.

---

## 1. PARTLY WRONG, corrected 2026-09-15: the container was mostly fine

**The audit claimed the Gazebo image could not run any downstream node, naming
five missing packages. Four of the five were present. This section is kept, and
corrected, rather than deleted.**

What was actually true, measured in the built image with `apt-mark`:

```text
manual: ros-jazzy-image-geometry     <- genuinely absent, now added
auto:   ros-jazzy-cv-bridge          <- already present, pulled in transitively
auto:   ros-jazzy-message-filters    <- already present, pulled in transitively
        python3-opencv, numpy        <- already present
```

`cv_bridge`, `message_filters`, `python3-opencv` and `numpy` all import in the
image and always did; they arrive as dependencies rather than as explicit apt
lines. Genuinely missing were **`image_geometry`**, which
`igvc_lane_detection`'s `PinholeCameraModel` needs, and **Nav2**, without which
the ground-truth lane grid can be published but nothing can act on it. Both are
now installed in `docker/Dockerfile.gazebo-jazzy`.

**How the error was made, because the method matters more than the fact.** Two
critics asserted the packages were missing. The claim was then "verified" by
`grep` against the Dockerfile's explicit `apt-get install` list. That grep can
only establish **not explicitly listed**, which is a different claim from **not
present**, and the two were conflated. A transitive dependency satisfies the
second while failing the first.

The check that would have settled it in one command, and the one to use next
time, runs against the built image rather than the recipe:

```bash
docker run --rm igvc-gazebo-jazzy:latest python3 -c "import cv_bridge"
```

The general lesson is this project's own rule, applied to itself: **a test has
to be able to fail in the right direction.** Grepping a Dockerfile cannot
distinguish absent from inherited, so it was never capable of confirming the
claim it was used to confirm.

**What survives.** Checking the container can run a node *before* designing
around it is still right, and Nav2 and `image_geometry` really did have to be
added, so the rebuild the audit called for was necessary. The conclusion held
while the evidence for it did not.

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

~~Nothing above was applied to the repo. Another agent holds that tree.~~

**Applied 2026-09-15.** What this audit got right, wrong, and missed:

- **Item 2, the frame typo: right, and the most valuable thing here.** Fixed in
  `gazebo_sim.urdf.xacro`. `bringup_smoke_test.sh` now resolves the frame id
  off the image through `tf2_echo`, so the class of bug cannot return silently.
- **Item 6, "do not write a shim node": right about the cameras, wrong in
  general.** The bridge's YAML form renames topics with no node at all, which
  is cheaper than even this audit expected. But odometry needed one anyway, for
  a reason item 5 half-anticipated: Gazebo's odom frame is rotated by the spawn
  yaw, `gt_nav_bridge_node` reads odometry as a map-frame pose with no TF
  lookup, and therefore no amount of renaming or remapping can fix it. See
  `GAZEBO_SETUP.md` 9.3 for the measurement.
- **Item 6, "`igvc_simulation_interface` is the wrong home": right.**
  `gazebo_odom_shim` lives in `igvc_test_bringup`.
- **Item 5, "`/lane_ground_truth` may not be simulator-independent": the
  caution was justified and the conclusion was not.** The node itself is
  simulator-independent - it reads JSON and subscribes to nothing. What is not
  independent is the frame its grid is published in, which is the odom
  rotation above.
- **Item 7: right.** The smoke test was updated in the same change.
- **Item 3, `camera_info`: verified present** and carrying data.
- **Item 1: still the best part of this document,** because it is about method
  rather than fact.

**What it missed,** and what a future audit should look for: nothing here
checked whether two simulators were running at once. Six `gz sim` processes
produced a robot that circled at a barrel and a Nav2 lifecycle abort, both of
which read as navigation bugs and were neither. See `GAZEBO_SETUP.md` 10.5.
