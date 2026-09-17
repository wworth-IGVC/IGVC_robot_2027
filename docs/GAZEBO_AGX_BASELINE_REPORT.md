# Gazebo against the 2026 robot as designed, AGX Orin baseline

**Phase 0 only. Written 2026-09-17. Awaiting approval before any Phase 1 work.**

The governing assumption for this whole task: the 2026 robot runs its **NVIDIA
Jetson AGX Orin**, and the 2026 stack is not modified. Every latency and frame
rate below was measured on **x86, Windows 11, RTX 5070 Ti Laptop (Blackwell
sm_120), Docker Desktop 29.7.2, WSL2 Ubuntu-26.04**. None of them is a
statement about what the Jetson would do.

---

## 0. Read this first: four things that are not true

Phase 0 was supposed to confirm the starting state. It did, but it also
overturned four load-bearing beliefs. Each one would have cost a session.

1. **"Four FollowPath aborts in 143 s" is a misread instrument.** The number
   comes from `/navigator/status`, which prints `aborts={self._consecutive_aborts}`
   (`navigator.py:1702`). That counter is `min(x + 1, 4)`, it **saturates at 4**
   (`navigator.py:1595`, `:1644`, `:1684`) and **resets to 0 on success**
   (`:1675`). It also counts goal *rejections*, and explicitly does **not**
   count `STATUS_CANCELED` ("expected when we replan"). So `aborts=4` means "at
   least four consecutive failures at the instant it was sampled". The true
   total is unknown. My run today printed `aborts=1`. Nothing in
   `autonomy_check.sh` counts aborts at all, it echoes that string once
   (`:99-100`).

2. **The robot's camera and its planner are looking at two different courses.**
   `generate_igvc_world.py:137-138` paints a **constant 12 ft (3.658 m)** lane
   into `igvc_course.sdf`. Nav2's corridor does not come from that file at all, 
   `track_ground_truth_node.py:339-374` derives it from contours in
   `IGVC_track_generator/track.png`, which measures **2.766 m to 6.068 m**
   (median 4.431 m). Median disagreement **0.864 m**, maximum **2.410 m**, and in
   **34.9%** of samples the planner's corridor is *narrower* than the lane
   painted in Gazebo. **No ROS node ever reads `igvc_course.sdf`.** Any
   camera-versus-ground-truth comparison is currently comparing two courses.

3. **`lane_eval_node` cannot produce a non-zero score, by construction.** It
   thresholds both grids at `occupied_threshold: 50` (`lane_eval_node.py:25`,
   `:145`). But `track_ground_truth_node` writes **0 for the free corridor and
   -1 for everything else** and never writes a value ≥ 50 (`:330` `np.full(-1)`,
   `:374` and `:408` set 0; the comment at `:370-373` is explicit). Meanwhile
   `lane_detection` writes **100 for lane lines** (`lane_detection.py:199`,
   comment at `:57`). The two are *complementary encodings of different things*:
   ground truth marks drivable space, the detector marks paint. `gt_occ` is
   therefore always all-False, giving IoU 0.0, precision 0.0, recall 1.0,
   always. Verified by reading both files today.

4. **The P0 item names the wrong node.** `GAZEBO_TODO.md` and `STATUS.md:24`
   say the Gazebo image has no torch "so `lane_detection_node` cannot start".
   `lane_detection.py` imports **no torch** (grep count 0); it is classical CV, 
   CLAHE, HSV gate, Canny, `HoughLinesP`. The node that hard-fails without torch
   is **`lane_segmentation_node`** (YOLOPv2). Related: `STATUS.md:25` says
   `num_cameras: 3` makes "the synchroniser never fire and log nothing".
   `lane_detection.py:81-91` builds **one independent synchroniser per camera**,
   so camera 0 fires normally on the front camera; only 1 and 2 starve. And it
   **does** log, a 2 s watchdog with a throttled WARN (`:119`, `:924-929`).

---

## 1. Regression baseline, measured today

All three gates pass. These are the numbers any later phase must not regress.

| Check | Result | Evidence |
| --- | --- | --- |
| `render_check.sh` | **PASS**, exit 0 | `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, Mesa 25.2.8; camera and gpu_lidar both `PUBLISHING` |
| `bringup_smoke_test.sh` | **18 passed, 0 failed**, exit 0 | odom 7.796 m vs gz truth 7.804 m; heading disagreement **0.13°**; distance ratio **0.9989** |
| `autonomy_check.sh` | **PASS**, exit 0 | 51.3 m driven, span 23.1 m, centreline deviation mean 0.54 m / max 1.54 m, on the slab throughout, `aborts=1` |

Environment: Docker 29.7.2, 24 CPUs, 16.4 GB to the VM. **123.6 GB free on C:**
, well above the 40 GB floor. `docker system df`: 68.9 GB images (33.6 GB
reclaimable), 29.1 GB build cache.

### Two de-risking probes I ran because they change the plan

**torch on Blackwell, PASSES.** `igvc-humble-fused-drive`: torch
**2.14.0+cu130**, CUDA 13.0, cuDNN 9.24, `is_available: True`, device
`NVIDIA GeForce RTX 5070 Ti Laptop GPU`, capability `(12, 0)`, and
`arch_list` contains **`sm_120`**. A real convolution ran on the GPU:
`CONV_FORWARD_OK (1, 16, 224, 224) sum=24495.5254`. ultralytics 8.4.146,
cv2 4.5.4, numpy 1.26.4. **`onnxruntime` is absent.**

**Cross-container plumbing, better than expected.** Both containers sit on
`igvc_robot_2027_default` (172.19.0.2 and .3), DNS resolves between them, and
both set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0` and
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. That last one disables shared memory, which
pre-empts the exact silent failure the task warns about (`ipc=private` on both,
so they never shared an IPC namespace anyway). **One asymmetry:** only
`igvc_humble_fused_drive` sets `FASTDDS_DEFAULT_PROFILES_FILE`; neither Gazebo
service does. Still untested: whether **Humble and Jazzy** actually interoperate
over the wire. That is gate 3.1.3 and it stays a gate.

### Two starting-state defects found while establishing the baseline

- **`render_check.sh` never sources ROS.** `gz` on this image comes from the ROS
  *vendor* packages (`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`, gz-sim
  **8.15.0**), and `/root/.bashrc` does not source `setup.bash`. So the
  documented command, `bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh`
 , fails on a clean container with `gz: No such file or directory` and reports
  `RESULT: UNKNOWN`. One-line sim-side fix.
- **`sim_cameras` validates nothing.** `gazebo_sim.urdf.xacro:209,212` are two
  `xacro:if`s with no else. Any typo (`fron`, `front,left`, `true`) silently
  produces a robot with **zero cameras** that launches cleanly.

---

## 2. Deployment map, and the topology contradiction, resolved

The contradiction was: the design report says the Jetson is the brain and sends
CAN; `SIM_WORK_RESUMPTION.md` infers a laptop because `igvc_jetson_stack` is
commented out.

**Both were reasoning from a file that did not launch the competition run.**
`upstream/more_diverging_changes:scripts/auton_launch.sh:3-4` runs
`docker start dazzling_easley` then `docker exec -it dazzling_easley`.
`dazzling_easley` is a Docker-generated random name, created by a bare
`docker run`, outside compose entirely. No compose file on any branch declares
it; all four declare fixed `container_name` values. **Which service is commented
out proves nothing about what ran.**

The evidence that does discriminate, all of it pointing at the Jetson:

| Evidence | Where |
| --- | --- |
| The **same container** ran the 3× ZED X (GMSL2/Argus) **and** the CAN drive stack. One container is one machine, and GMSL2 cameras plug into the Jetson's capture card. | `auton_launch.sh:3,9` and `sensor_launch.sh:3,9` |
| `auton_launch.sh:9` passes no `hardware_interface`, so it defaults to `CanInterface` → ODrive plugin on `can0` | `igvc_fused_drive.launch.py:293`, `ros2_control_info.urdf.xacro:11-12` |
| A **systemd unit pokes Jetson AGX Orin PADCTL registers** then brings up `can0` at 250 kbit/s. systemd runs on the *host*. Added 2026-05-24, five days pre-competition. | `startup/devmem-init.service:8-14` |
| I2C described as "Jetson header pins 27 SDA / 28 SCL" | `sensor_launch.launch.py:391` |
| `more_diverging_changes` is configured for the **venue**: `goal_lat 42.400510946 / goal_lon -83.130518968` = Oakland University, Rochester Hills, MI. `main` has no GPS-waypoint launch file at all. | `nav2_gps_waypoint.launch.py:89-93` |
| 55 commits in the competition window; **54** on `more_diverging_changes` / `test_comp_changes_isaac_smi`, exactly **one** on `main` (unrelated `ping_location`). Operator scripts written mid-event, Sat 30 May 13:15 EDT. | `git log --all --since=2026-05-28 --until=2026-06-03` |

**The laptop hypothesis' main argument does not survive.** It rested partly on
no compose service granting CAN via `devices:`. Both CAN implementations use
**SocketCAN**, which is a *network* interface, not a `/dev` node
(`can_interface.cpp:305` `socket(PF_CAN, SOCK_RAW, CAN_RAW)`). `network_mode:
host` plus `privileged: true`, which both active services have, is exactly
and sufficiently what SocketCAN needs.

**And a supporting claim in our own docs is wrong.** `SIM_WORK_RESUMPTION.md:179-181`
and `CLAUDE.md` both say "Both files use `network_mode: host` with a shared
FastDDS/Zenoh profile, so this was one ROS graph across two machines."
`docker-compose.yml:20` sets `rmw_fastrtps_cpp`; `docker-compose.jetson.yml:31`
sets `rmw_zenoh_cpp`. Two different RMWs do not interoperate, and
`zenoh_config.json5` listens only on `tcp/localhost:7447` with the cross-host
endpoint commented out. **As configured they could not have formed one ROS
graph.** The jetson compose header still claims FastDDS; `b15300f` swapped the
RMW on 2026-05-16 and never updated the comment.

### The map

| Function | 2026 robot, as designed | Sim today | Sim after this task |
| --- | --- | --- | --- |
| 3× ZED X, LiDAR, GPS | Physical sensors on the Jetson. **Confidence: high.** | Gazebo sensors in `igvc-gazebo-jazzy`, renamed onto the ZED namespace by `gazebo_bridge.yaml` | Same, plus `/fix` from a Gazebo `navsat` sensor |
| Lane detection | Jetson (`lane_segmentation_node`, YOLOPv2). **Report names YOLOPv2 once, with no resolution and no frame rate.** | Not running | `igvc-humble-fused-drive` as the Jetson stand-in |
| Obstacle detection | **See §3, the report and the code disagree.** | Not running; obstacles come from `track_points.json` | Nav2 stock `ObstacleLayer` on bridged `/scan` + clouds |
| Odometry | ZED depth + IMU + LiDAR fused with GPS "using a ZED ROS 2 wrapper" (report p.15), technically implausible, see §3 | `gazebo_odom_shim` rotating Gazebo odometry by the spawn yaw | Unchanged |
| Nav2 + IGVC navigator | Jetson. **Confidence: high**, same container as the drive stack. | `igvc-gazebo-jazzy` | Unchanged |
| Drive | `diff_drive_controller` + ODrive `CanInterface`, CAN from the Jetson | Gazebo DiffDrive plugin, no ros2_control | Unchanged pending RQ-06 |

**Reconciling Aidan's answer.** He said "the Dockerfile on `main`". Given `main`
got one unrelated commit during the event while `more_diverging_changes` got 54,
both can be true of different things: **the image may have been built from
`main`'s Dockerfile while the code bind-mounted into it was
`more_diverging_changes`.** That reconciles his answer with the history rather
than overruling it, and it is worth putting to him in exactly those words.

---

## 3. Design report versus code

The report's author (Nitin Chandrasekhar, team lead) is the author of every
competition-window commit (`nchan18`). These are not third-party claims.

| Report says | Code says |
| --- | --- |
| Obstacle detection: a custom **13-class** ONNX through the ZED wrapper (p.15) | The model has **15 classes**. Parsed from `best.onnx` metadata: `{0:'3', 1:'Tire', 2:'pedestrianCrossing', 3:'person', 4:'signalAhead', 5-9: speedLimit20/25/30/35/40, 10:'stop', 11:'stopAhead', 12:'tire', 13:'traffic barrel', 14:'yield'}`. Ultralytics **YOLOv12n**, imgsz 640. Class 0 labelled `'3'` and `Tire`/`tire` duplicated look like labelling bugs. |
| …loaded through the ZED ROS 2 wrapper | **Nothing in the repository consumes the ZED wrapper's detection output**, zero subscribers to `obj_det`/`ObjectsStamped`/`zed_msgs` across all 16 branches. `od_enabled: false` in both configs the rest of the repo uses. The one enabled config, `zed2i_custom_detection.yaml`, is referenced by a launch file that exists **only on branch `2026/zed`**, and its `custom_onnx_file` points at `/home/root/ros2_ws/.../best.onnx`, a path that does not exist and is malformed (`/home/root`). |
| LiDAR data "fused with GPS using a ZED ROS 2 wrapper" (p.15) | The ZED wrapper has no LiDAR ingest. The LiDAR is a USB/J40 peripheral in the schematic. **The report's most implausible claim.** |
| "negligible difference" between sim and reality (§8.5) | Three lines of text, zero data, no sim-vs-real measurement. It was a claim about Isaac streaming HD1200 into the real ZED SDK. It does not transfer to Gazebo's 640×360 pinhole RGB. Quoting it for Gazebo would be a category error. |
| RPLIDAR **C4** (p.12) | **C1** throughout: `rplidar_c1.urdf.xacro`, `top_rplidar_c1_link`, sensor `rplidar_c1` |
| Self-Drive functions: **sixteen** (§7.2) and **twenty-one** (§1.2.3, §7.2) | Both appear, four lines apart, plus "sixteen completed in one run" which equals the total asserted elsewhere. |
| Nav2 with "SmacHybrid Planner and Regulated Pure Pursuit" (p.15) | Planner is `SmacPlanner2D` (2D, not Hybrid); controller is RegulatedPurePursuit. But the report's *mechanism* description, sampling carrot distances over a distribution and scoring candidates on speed/obstacle-distance/progress, describes DWB or MPPI, not RPP. |
| No wheel radius, no resolution, no frame rate anywhere | Grep across all 19 pages: `radius` 0 hits, `fps`/`Hz` 0 hits, `resolution` 0 hits. The report gives RQ-02 no help at all. |
| Cover: "Date Submitted: May 15, 2025"; body: "the 2026 competition rules" |, |
| §5.2.2: "Images will be attached below" showing detection results | `p.15` and `p.16` contain **zero embedded images**. |

**What this means for the added obstacle-detection item.** The real
detection-to-costmap node is `obstacle_costmap.py`, which consumes
`/yolo/detections_3d` from **`yolo_ros`, not the ZED SDK**, and it exists only
on `2026/more_diverging_changes` and `2026/test_comp_changes_isaac_smi`, not on
`main`. On `main`, obstacles reach the costmap as **raw sensor data through
Nav2's stock `ObstacleLayer`** (`nav2_lane_follow_config.yaml:251-258`,
`observation_sources: "front_pointcloud left_pointcloud right_pointcloud lidar_scan"`,
`enabled: true` in both costmaps). Useful fact: `src/yolo_ros/best.pt`
(5,530,138 bytes, the same 15-class model) **is already on disk in this
checkout**, because `main` pins the submodule at `fc9679b`.

---

## 4. Robot width: four different numbers, and the abort mechanism

The to-do list says "a 0.87 m gap for a 0.70 m robot". `GAZEBO_SETUP.md` 10.4
already retracts the 0.87 m. The 0.70 m needs retracting too.

| Source | Width |
| --- | --- |
| **URDF collision geometry**, chassis mesh bbox under `rpy yaw +90°` | **0.810 m** (`test_robot_body.urdf.xacro:203-208`) |
| Nav2 footprint (both costmaps) | **0.700 m** (`nav2_lane_follow_config.yaml:250`, `:354`) |
| collision_monitor StopZone | 0.800 m (`:176`) |
| collision_monitor SlowZone | 1.000 m (`:169`) |
| Design report | 0.762 m (2.5 ft) |

The **chassis** is the widest part, not the wheels, the wheels are ~0.17 m
inside the chassis envelope (outer faces at +0.3164 / −0.3239). So Nav2 plans
and inflates for a robot **11 cm narrower than its own collision model**.
`GAZEBO_SETUP.md:660`'s "the robot is 0.70 m wide" promoted a config value to a
measurement.

**The abort mechanism, arithmetically.** At the tightest barrel in the geometry
Nav2 actually plans in (`track.png`): 1.767 m clear, minus
`obstacle_inflate_radius_m` 0.30 = 1.467 m, minus the 1-cell lethal lane-boundary
rim `gt_nav_bridge_node` stamps = 1.367 m of free cells. Nav2 then inflates both
sides: 1.367 − 2 × 0.35 (inscribed radius) = **0.667 m** below
`INSCRIBED_INFLATED_OBSTACLE`, and 1.367 − 2 × 0.75 (inflation_radius) is
**negative**, *there is no zero-cost cell anywhere in that gap*. A 0.810 m
robot does not fit in 0.667 m; a 0.700 m one barely does.

Widening the footprint to the true 0.81 m raises the inscribed radius to 0.405
and **makes this worse before it makes it correct**. That is still the right
trade, but it is a deliberate decision, not a drive-by fix.

**Three config comments that contradict their own values** (all in
`nav2_lane_follow_config.yaml`, none of which I will edit, they are real-robot
config):

- `:315-316` "lethal_cost_threshold is set **ABOVE 100** so lane cells come in
  as high-cost (253) rather than lethal (254)", the value on `:325` is **90**,
  and `gt_nav_bridge_node.py:274` writes lane cells as exactly **100**. So
  100 ≥ 90 and they become **LETHAL (254)**, precisely the failure the comment
  says forced `use_collision_detection: false`. The same comment reasons about
  MPPI; the configured controller is RegulatedPurePursuit.
- `:331-339` titled "Reduced from 0.75m", reasons to "0.50m", and the value two
  lines later is **0.75**.
- `:363` sizes the global obstacle ranges "to match the 30×30 m window"; the
  global costmap is **100 × 100 m**.

Also worth knowing: **collision_monitor's point-cloud sources are all
`enabled: false` in the shipped real config** (`:189`, `:197`, `:205`), so its
stop and slow polygons can never trip, on the real robot, not just in sim.

---

## 5. Status table

`VERIFIED` = a command ran and a number came back. Nothing here is `VERIFIED`
yet except the baseline, because Phase 0 stops before the work.

| # | Item | Status | One-line reason |
| --- | --- | --- | --- |
|, | Regression baseline (render / smoke / autonomy) | **VERIFIED** | 18/18, autonomy PASS, GPU confirmed, §1 |
|, | torch on Blackwell | **VERIFIED** | `sm_120` present, real conv ran on GPU, §1 |
| P0-1 | Get perception running | **PLANNED** | Path is clear; the to-do list names the wrong node (§0.4) |
| P0-2 | `num_cameras` sim override | **VERIFIED, not needed** | `/lane_map` carries 253 occupied cells with camera 0 alone; one startup WARN only (§10.7) |
| P0-3 | Camera point cloud orientation | **PLANNED** | No prior research exists on this; pass 2 never discusses optical frames |
| P1-4 | RQ-06 control path | **TEAM DECISION** | Brief to write; pass 2 recommends `gz_ros2_control` |
| P1-5 | FollowPath aborts | **PLANNED, premise corrected** | The "four" is a saturating gauge (§0.1); corridor is a *hypothesis*, not a diagnosis (§9 C-3) |
| P1-6 | `render_check.sh` on other machines | **OUT OF REACH** | Runbook only, no access to the 5080 or the Win10 machine |
| P1-7 | Verify `igvc_gazebo_linux` | **OUT OF REACH** | Runbook only, no Linux host |
| P1-8 | RQ-11 barrel physics | **TEAM DECISION** | Brief to write |
| P1-9 | Publish `/fix` | **PLANNED** | Blocked on a datum decision and a topic-name decision (§6) |
| P2-10 | `/cmd_vel` timeout | **PLANNED** | Smoke test already reports "DiffDrive held the last command" across 4 s |
| P2-11 | Four zero-normal meshes | **PLANNED** | Staging-directory repair + hash comparison |
| P2-12 | `sim_cameras:=all` on the real course | **PLANNED** | Blocked by a launch-file gap (§6) |
| P2-13 | YOLOPv2 at 640×360 | **PLANNED, premise corrected** | It will never crash, the frame is upscaled to 1280×720 first |
| P2-14 | `wheel_radius` | **DOCUMENT ONLY** | Sim corroborates 0.1016; ratio 0.9989 today |
| ADD-15 | Obstacle detection inventory | **VERIFIED (inventory)** | §3, 15 classes not 13, nothing consumes ZED detections, `best.pt` already on disk |

---

## 6. The plan

Order, the test for each, the number I expect, and what would make it fail.

### Phase 1, P0

**1.1 Cross-container DDS (gate).** Run the sim in `igvc_gazebo`, run a counter
in `igvc_humble_fused_drive`, count messages on the front image, depth,
camera_info, `/clock`, `/tf`, `/tf_static` **in both containers simultaneously**.
*Expect:* Humble-side rates within 10% of Jazzy-side. *Fails if:* Humble/Jazzy
do not interoperate, or bridge multicast does not cross the Docker bridge.
*Fallback if it fails:* set `FASTDDS_DEFAULT_PROFILES_FILE` on the Gazebo
service too, then static peers. **This is the gate for everything else in P0.**

**1.2 A sim perception launch file.** New file
`src/igvc_test_bringup/launch/gazebo_lane_test.launch.py`, sim-side, copying the
override pattern that already exists twice in the repo
(`lane_seg_zed2i_test.launch.py:70-86` puts the yaml first and the dict second).
This is how `num_cameras` gets overridden **without touching
`lane_detection_config.yaml`**, which is forbidden. `lane_follower.launch.py`
has the dict *before* the yaml, so the yaml wins and nothing is overridable, 
which is why a new file is cleaner than editing that one.

**1.3 Point cloud orientation.** Barrel at a known world pose; transform the
cloud into `base_link` through its own `header.frame_id`; assert the fitted
floor normal is within 5° of +Z and the barrel clusters at the expected range.
*Prove it can fail* by running once with a deliberately wrong frame.
*Expect:* either clean pass, or a 90° error, `gz_frame_id` is the optical
frame, which is right for the rasters and may be wrong for the cloud.

**1.4 Lane output, and this one needs a decision.** As specified, it cannot
work: §0.2 (two different courses) and §0.3 (complementary encodings). My
proposal, all sim-side: a new `scripts/gazebo/lane_eval_sim.py` that renders
lane-line ground truth from **the same `track_points.json` geometry
`generate_igvc_world.py` paints into the world**, and scores `/lane_map`
against that. It reuses the exact geometry the camera sees, which is the whole
point. `lane_eval_node.py` is in `src/igvc_lane_detection/` and stays untouched.

### Phase 2, P1

**2.1 FollowPath aborts.** First *instrument* it: count `STATUS_ABORTED` results
from the log rather than sampling a saturating gauge, and log pose + nearest
obstacle + costmap cost along the path at each. Then ≥3 runs per condition
(single runs have misled this project twice). The corridor arithmetic in §4 is
already the hypothesis. All hypothesis tests go through a clearly-named sim-only
overlay; `nav2_lane_follow_config.yaml` is not edited.

**2.2 `/fix`.** `<spherical_coordinates>` into the **generator**, plus a
`navsat` sensor on `front_gps_antenna_link` (which is at `0.177 -0.133 0.218`
relative to `base_link`, note the antenna is 13.3 cm right of centreline, a
real lever arm), plus a bridge row. *Test:* drive a known displacement, compare
NavSatFix-derived local ENU against Gazebo ground truth including heading, so a
rotated datum fails. **Trap:** `igvc_course.sdf` is *committed* and only
regenerated when `track_points.json` is newer (`start_sim.sh:70`), so the
regenerated world must be committed in the same change or nobody's checkout
changes.

**2.3 and 2.4**, RQ-06 and RQ-11 briefs under `docs/decisions/`.
**2.5**, runbooks (kept locally, not in this repo, see 16.8) for the 5080 laptop, the Windows 10
machine, and native Linux.

### Phase 3, P2

`/cmd_vel` timeout with ≥5 repeats reporting a distribution, judged on Gazebo
**ground-truth pose** with an asserted on-slab precondition. Mesh repair into a
staging directory with hashed vertex/topology comparison. `sim_cameras:=all` at
several resolutions. `wheel_radius` documented only.

### Three decisions I need from you before Phase 1

1. **The lane-evaluation approach** (§0.3 / 1.4). My recommendation is the new
   sim-side evaluator. The alternative, making `/lane_ground_truth` emit 100
   for lane cells, is a forbidden edit *and* riskier, because
   `gt_nav_bridge_node` and Nav2's StaticLayer both read the 0/-1 convention.
2. **Where the updated docs go.** The other agent moved `GAZEBO_TODO.md` and
   `DOCKER_CHANGES.md` into `docs/archive/`, whose README says archived docs
   "are not maintained". The deliverables ask me to update both. I propose
   updating **`docs/STATUS.md`** (the live status doc) and bringing the to-do
   list back to `docs/GAZEBO_TODO.md` rather than editing archived files.
3. **Which branch.** The repo is on **`docs-trim`, 5 commits ahead of `main`
   and unpushed**. I have not been told about this branch. Do I work on it,
   or branch from `main`?

### Risks

- **The lens question is the single highest-value unknown.** ZED X 2.2 mm gives
  104.63° HFOV; 4 mm gives 74.06°. The sim uses **100°**. So the sim is either
  4.6° too narrow or **26° too wide**, the sign is unknown. Every sim-vs-real
  camera claim depends on it. One `camera_info` echo from a bag settles it.
- **YOLOPv2 will pass while degraded.** `yolopv2_infer.py:114,213` resizes every
  frame to **1280×720** before letterboxing, so a 640×360 Gazebo frame is
  interpolated up and the tensor shape is identical to the real path. It cannot
  crash. Judge it by lane-point output, never by the node staying alive.
  Separately, 13 tuning parameters in `lane_segmentation_config.yaml` are
  denominated in **pixels** and were tuned at the real resolution; at 640×360 an
  area threshold is effectively 5.76× stricter.
- **`gazebo_nav_test.launch.py` cannot set camera resolution.** It forwards only
  `world, track_file, sim_cameras, headless, rviz, use_sim_time` (`:142-149`).
  So the autonomy path is locked to 640×360 at 15 Hz, which blocks exactly the
  experiment that answers P2-13. Sim-side launch-file fix, permitted.
- **The camera budget was measured at the wrong FOV.** `three_camera_load.sdf`
  uses `1.7` rad (97.40°); the URDF uses `1.7453` (100.00°). The poses match
  exactly, so this looks like an oversight.
- **Camera index order differs between perception and driver config.** Perception
  orders front/left/right; every ZED launch orders left/front/right. `idx == 0`
  means "front" in one and "left" in the other, and `zed_multi.launch.py:166-169`
  grants TF publishing to `idx == 0`. Harmless today because perception keys off
  topic names.
- **Docker disk.** 123.6 GB free now, and no rebuild is planned. If one becomes
  necessary I will stop and ask, per the rules.

---

## 7. Questions for Aidan, each answerable in one line

1. **On the robot's Jetson, run `docker ps -a`, is there a container called
   `dazzling_easley`?** That name is hardcoded in all five of Nitin's
   competition scripts. Its existence settles the topology in thirty seconds
   and does not depend on anyone remembering June correctly.
2. You said "the Dockerfile on `main`" ran at competition. Could the **image**
   have been built from `main` while the **code** bind-mounted into it was
   `more_diverging_changes`? That would reconcile your answer with the commit
   history rather than contradict it.
3. **Which lens is fitted to the three ZED X units, 2.2 mm or 4 mm?** (Or: does
   anyone still have a competition bag with a `camera_info` in it?)
4. Is `wheel_radius: 0.2032` correct on the real robot, or is it the diameter?
   The simulator corroborates 0.1016 to 0.11% again today.
5. Was `startup/devmem-init.service` actually installed and enabled on the
   Jetson?
6. RQ-06: `gz_ros2_control` in-process, or a `GazeboDriveHardware` topic bridge?
7. RQ-11: should barrels tip, or stay static?
8. Did the 2026 team record a course origin (lat/lon) anywhere? If not I will
   pick an arbitrary datum for `<spherical_coordinates>` and label it as such.

---

## 8. Retractions and corrections made in Phase 0

Kept visible on purpose, per the project's working agreements.

1. "Four FollowPath aborts in 143 s", **withdrawn**, it is a saturating gauge.
2. "The robot is 0.70 m wide", **withdrawn**, that is the Nav2 footprint; the
   collision model is 0.810 m.
3. "`lane_detection_node` cannot start without torch", **withdrawn**, wrong
   node; it imports no torch.
4. "The synchroniser never fires and logs nothing", **withdrawn**, one
   synchroniser per camera, and there is a watchdog that logs.
5. "One ROS graph across two machines" (`CLAUDE.md`, `SIM_WORK_RESUMPTION.md`), 
   **withdrawn**, the two compose files set different RMW implementations.
6. "The design report describes a 13-class model", the model has **15**.

---

## 9. Corrections to Phase 0

Raised on review, 2026-09-17. The original text above is left as written; this
section is the amendment record. Three of the nine corrections overturned my
finding, one overturned the challenge, and the rest sharpened wording.

**C-1. The GPS waypoint is not the competition venue. My error.**
`nav2_gps_waypoint.launch.py:89-93` defaults to `42.400510946, -83.130518968`.
Oakland University is near `42.6725, -83.2175`. Great-circle distance: **31.1 km**,
computed today. That is the Detroit area, not the venue. **Remove that row from
the topology evidence**, and do not use those coordinates as a datum labelled
"venue". The topology conclusion in §2 stands without it: it rests on the shared
`dazzling_easley` container, the `CanInterface` default, the Jetson PADCTL
registers in `devmem-init.service`, the GMSL2 camera attachment, and the
SocketCAN point.

**C-2. "`lane_eval_node` cannot score, by construction" is true of today's code
only, and the history is worse than the bug.** Full archaeology:

| Commit | When | Corridor fill |
| --- | --- | --- |
| `5ca204f` "working test cases" | 2026-05-14 05:58 | **100** |
| `37832a1` "ouput" | 2026-05-14 **11:12** | **0** |
| `1e18251` "Test cases (#10)" | 2026-05-22 | 0 |
| `a8e316f` "spelling" | 2026-06-01 | 0 |

The encoding changed **five hours and fourteen minutes** after the file first
appeared, at both write sites together, and the commit's own added comment gives
the motive: Nav2's `lethal_cost_threshold >= 90` and `_extract_centreline`'s
`data == 0` both need the corridor to be **free**, not occupied. Nobody updated
the evaluator. `lane_eval_node.py` is a **single blob identical on every ref that
has ever existed**, and `occupied_threshold` was never anything but 50. The
100-fill blob exists on no branch tip; it is reachable only as history.

Also, my "recall 1.0" was a zero-division convention, not a result:
`lane_eval_node.py:158` reads `recall = float(tp / (tp + fn)) if (tp + fn) > 0 else 1.0`.
With an empty ground-truth set, `tp + fn == 0`, so it returns 1.0 without
dividing.

**And design report figure 7 was produced by the 100-fill code.**
`debug_gui/lane_compare_ui.py:32-37` colours cells `>= 50` cyan and `0 <= arr < 50`
grey. The figure's ground-truth panel is **cyan**, so it predates 2026-05-14 11:12.
Its printed metrics are `IoU=0.083, Precision=0.110, Recall=0.253,
Latency=66.7 ms, Dropped=1853/1915`: **96.8% of frame pairs discarded**, about 62
pairs actually scored. The two panels are disjoint in two independent ways: the
ground truth is a thick filled corridor band confined to the upper-left, the
prediction is thin painted lane lines centred and filling the panel. A filled
corridor and the lines bounding it are near-disjoint sets by construction, so
even a perfect detector scores near zero against that ground truth. **Section 8.5
cites this figure as evidence of simulation fidelity. It is the only quantitative
sim-fidelity evidence in the section, and it measures a units mismatch.**

**C-3. My abort arithmetic double-counted the robot's width. Corrected.**
The 0.667 m band already has the inscribed radius removed from both sides, so it
is the room available to the robot's **centre**, not to its body. Comparing the
robot's full width against it counts the width twice. Corrected: with the true
0.405 m half-width, the room is `1.367 - 0.81 = 0.557 m`, which is **positive, so
the robot fits**. What does still hold is the weaker, and still relevant,
statement: **there is no zero-cost cell anywhere in that gap**, because
`1.367 - 2 x 0.75` is negative. **"Mechanism diagnosed" is downgraded to
"hypothesis"** in the status table.

**C-4. The wheel inset was a two-sided figure quoted as one-sided.**
"~0.17 m inside the chassis envelope" is the total across both sides. Per side it
is 0.081 m (left) and 0.089 m (right). Separately, the outer faces at `+0.3164`
and `-0.3239` put the wheel track centre **3.75 mm to the robot's right of
`base_link`**. For odometry this is second order and below the noise the sim
already shows: a 3.75 mm asymmetry on a 0.576 m separation is 0.65%, while today's
measured distance ratio is 0.9990 and heading disagreement 0.13 degrees. It would
matter for a dead-reckoning heading estimate over a long run; it does not affect
any Phase 1 conclusion.

**C-5. The "5.76x" claim was challenged and survives. The challenge was wrong on
both counts.** The masks are resized **back to the incoming camera resolution
before `infer()` returns** (`yolopv2_infer.py:249-254`, stated in the docstring at
`:200-203`), so nothing downstream ever sees the 1280x720 buffer and a threshold
applied after the resize does scale with the camera. And the missing factor of
2.4 is `pub_downscale_factor: 1.25` (`common_stereo_real.yaml:32,35,36`):
`1920 / 1.25 = 1536` and `1080 / 1.25 = 864`, so `1536 / 640 = 2.4` exactly,
`864 / 360 = 2.4` exactly, and the area ratio is **5.76** exactly.

Per parameter: **11 of the 13 scale with the incoming camera resolution** (10 on
the RGB-resolution mask, 1 on the depth image). `model_img_size` does not, being
applied to the fixed 1280x720 buffer at `yolopv2_infer.py:221-223`.
`depth_search_radius_px` is **dead in this node**: read at
`lane_segmentation.py:108` and never used again. So the correct statement is:
**area-denominated thresholds are 5.76x stricter at 640x360; linear ones
(kernels, strides, dilations) are 2.4x.** Two incidental defects found:
`lane_segmentation_config.yaml:138` documents `ll_dilation_px` as being in
"YOLOPv2 input resolution", which is false, and `:132-135` says
"Lowered from 150 -> 60 px" beside a value of 10.

**C-6. The lens FOV numbers are correctly derived, but I mislabelled them.**
Source: `isaac/exts/exts/sl.sensor.camera/sl/sensor/camera/utils.py:8-21`, which
gives focal lengths **in pixels** (741.6 and 1272.5 at 1920 wide), proven to be
pixels by `annotators.py:93-103` where the pixel size cancels. So
`HFOV = 2 atan(W / 2f)`: `2 atan(1920 / 1483.2) = 104.6278` and
`2 atan(1920 / 2545.0) = 74.0633`. The arithmetic reproduces exactly, and SVGA is
self-consistent.

**But the repo never writes "2.2 mm" anywhere.** The extension's keys are
`"standard"` and `"4mm"`. Mapping "standard" to the 2.2 mm SKU is external
knowledge and is **NOT VERIFIED** here. Further, these are the intrinsics
Stereolabs ships for a **simulated** camera, which may be rectified FOV rather
than raw lens FOV; that would plausibly explain a gap against a datasheet figure
nearer 110 degrees. I am not asserting a datasheet number. What settles it is one
`camera_info` from the rectified topic:
`HFOV = 2 atan(width / (2 * K[0]))`. That also yields the *rectified* FOV, which
is the number the simulator should match anyway.

**C-7. What the wheel radius evidence actually proves.** The sim's
`wheel_radius: 0.1016` is not a typed-in guess: `gazebo_sim.urdf.xacro:36-49`
derives it from the wheel **mesh bounding box**, which measures
`0.2032 x 0.0640 x 0.2032 m`, so 0.2032 is the diameter (exactly 8.00 in). The
joint chain corroborates it: at `r = 0.1016` the drive wheels contact at
`-0.2290` against the caster's `-0.2331`, a 4 mm difference that is build
tolerance, whereas `r = 0.2032` would put them at `-0.3306` and leave the caster
floating 97 mm in the air. So the sim proves **the robot description is
self-consistent only at 0.1016**, and that `controllers.yaml` disagrees with the
description. It does **not** prove either matches the physical wheel. That still
needs a tape measure. (See §11: `more_diverging_changes` already sets 0.1016.)

**C-8. Questions 1 and 5 assumed the 2026 AGX is reachable. It is not.**
Both are replaced by a single question in §12.

**C-9. A retraction of the done list, not just a defect.** `GAZEBO_TODO.md` lists
`render_check.sh` under "Done, and verified", and the quickstart tells a
teammate to run it as the first thing on a new machine. **It failed on a clean
container**, because it never sourced ROS and `gz` is not on PATH until you do.
Anyone following the documented setup on a new machine would have seen
`RESULT: UNKNOWN` and read it as a broken GPU. Fixed in Phase 1; recorded here as
a retraction because the item was marked verified and was not.

---

## 10. Phase 1: what was changed, and the evidence

Branch **`gazebo-phase1`**, created from `main`. See §13 for why, and for a
conflict with the docs instruction.

### 10.1 Baseline after the changes, rerun

| Check | Before | After | Evidence |
| --- | --- | --- | --- |
| `render_check.sh` | exit 0 **only if ROS was already sourced** | **exit 0 from a clean shell** | `which gz` beforehand prints `NOT ON PATH`; result `HARDWARE RENDERING (NVIDIA)` |
| `bringup_smoke_test.sh` | 18 passed / 0 failed | **18 passed / 0 failed**, exit 0 | odom 8.588 m vs truth 8.597 m, heading 0.13 deg, ratio **0.9990** |
| `autonomy_check.sh` | PASS | see §10.6 | rerun with the new clearance metric |

### 10.2 `render_check.sh` now sources ROS (C1)

`gz` on this image comes from the ROS **vendor** packages
(`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`, gz-sim 8.15.0) and
`/root/.bashrc` does not source `setup.bash`. One line added, with the
reasoning in a comment.

**Test that can fail:** run it from a shell with no ROS sourced. Before: `gz: No
such file or directory`, `RESULT: UNKNOWN`. After: `which gz` prints
`NOT ON PATH`, and the script still reports `HARDWARE RENDERING (NVIDIA)`,
exit 0.

### 10.3 `sim_cameras` now fails loudly (C2)

Two guards, because there are two ways in. `gazebo_sim.launch.py` raises a
readable `RuntimeError` before xacro runs; `gazebo_sim.urdf.xacro` validates via
`['none','front','all'].index(cams)` for anyone invoking xacro by hand.

**A wrong first attempt, worth recording:** the xacro guard initially did
nothing, because **xacro evaluates properties lazily** and a validation property
that nothing reads is never evaluated. The fix was to make the validated index
the thing the dispatch switches on (`cams_idx >= 1`, `cams_idx == 2`), so it
cannot be skipped. My first test run also gave a **false pass**: all six cases
exited non-zero, including the valid ones, because the workspace was not sourced
and the package was not in the ament index. It "rejected" the typo for the wrong
reason. Both caught by checking the actual error text rather than the exit code.

**Test that can fail**, after building and sourcing:

| Value | exit | rgbd cameras |
| --- | --- | --- |
| `front` | 0 | 1 |
| `all` | 0 | 3 |
| `none` | 0 | 0 |
| `fron` | **2** | rejected by the guard |
| `front,left` | **2** | rejected by the guard |
| `true` | **2** | rejected by the guard |

### 10.4 The world now paints the course the planner plans in (A1.2)

**The verdict is case (a): `track.png` is the designed course, and the constant
12 ft lane is the outlier.** Evidence:

- `IGVC_track_generator/constants.py:116-117` sets `TRACK_WIDTH_MIN_FT = 10`,
  `TRACK_WIDTH_MAX_FT = 20`, and `main.py:503` paints
  `width = MIN + (MAX - MIN) * (0.5 + 0.5 sin(4 pi i/N))`: a sinusoid, two full
  cycles per lap. There is **no constant lane width anywhere in the generator**.
- That is a direct transcription of IGVC 2026 rules II.2: "Track width will vary
  from ten to twenty feet wide", with the 500/120/100 ft course constants
  appearing verbatim in `constants.py:93-95`. **12 ft appears in neither.**
- Straights-only measurement of `track.png` (curvature radius >= 25 m, n = 327)
  matches the generator's intended width to a **median 0.85% relative error**,
  correlation **0.9974**, with **100%** of samples within 5%. Only **8.0%** are
  within 5% of 3.658 m. So the 2.766 to 6.068 m spread is **real design, not a
  curve-measurement artifact**.
- `_get_lane_polygon_from_image` is a faithful reader for this image: the PNG has
  exactly the 4-contour nesting its area-rank assumption needs. Two fragilities
  worth knowing: any extra white mark breaks the ranking silently, and
  `len(contours) < 4` falls through to a constant-width fallback with
  `lane_half_width_m` default 0.25, i.e. a silent 0.5 m corridor.

**The change.** `generate_igvc_world.py` now paints the variable profile by
default, mirroring the track generator's constants including its integer
truncation, so the Gazebo paint's **inner edge** lands on `track.png`'s inner
edge. The inner edge is the number that matters, because
`track_ground_truth_node` fills the corridor between the inner edges.
`--lane-width-ft` is kept as an explicit override for a constant-width debug
course. The regenerated `igvc_course.sdf` is committed in the same change,
because `start_sim.sh:70` only regenerates when `track_points.json` is newer and
would otherwise leave every checkout on the old world.

**Test that can fail:** measure the generated world's corridor against
`track.png` by ray-casting from 200 centreline points.

| World | corridor (m) | error vs `track.png` | within 0.15 m |
| --- | --- | --- | --- |
| **variable (new default)** | 2.734 to 5.778, median 4.210 | median **-0.043**, mean -0.063 | **90.0%** |
| constant 12 ft (old) | 3.541 to 3.639, median 3.592 | median **-0.813**, max 2.202 | **7.0%** |
| `track.png` reference | 2.752 to 5.814, median 4.399 | | |

The residual 0.538 m maximum is at curves, where an averaged-vertex-normal
offset and a disc-stamped raster necessarily differ. Median error improved by a
factor of about 19.

### 10.5 Camera resolution reaches the autonomy path (C4)

`gazebo_nav_test.launch.py` forwarded only six arguments, so
`sim_camera_width/height/hz` were unreachable on the one launch file that runs
the full stack, which blocked the "does YOLOPv2 tolerate 640x360" experiment
entirely. Now declared and forwarded. `--show-args` confirms all three, with
defaults 640, 360 and 15 so nothing changes unless asked.

### 10.6 Lane-boundary compliance, reported not enforced (C5)

`autonomy_check.sh` measured distance to the centreline only, which cannot say
whether the robot stayed inside a lane whose width varies by a factor of two.
It now also reports the **clearance between the chassis edge and the inner edge
of the painted line**, evaluated at the **local** half-width, and at both
candidate robot half-widths (0.405 m chassis collision, 0.350 m Nav2 footprint).
The width profile is imported from `generate_igvc_world.py` so there is one
definition. **Pass criteria are unchanged**, per instruction.

I confirmed the premise first: both painted lines are offset by the same
distance either side of `centerline_m`, so the centreline used for "deviation"
**is** the centre of the painted lane, by construction.

**The result contradicts the arithmetic that motivated this item, and the
milestone wording does not need retracting.**

Measured over two independent 100 s runs, 192 samples each. Run A reached a max
centreline deviation of **1.93 m**, worse than the 1.54 m the breach was
predicted against; run B reached 1.53 m. Neither crossed the paint.

| Robot half-width | Worst clearance, run A | run B | Samples over the line |
| --- | --- | --- | --- |
| 0.405 m (chassis collision) | **+0.254 m** | **+0.228 m** | **0%** |
| 0.350 m (Nav2 footprint) | **+0.309 m** | **+0.283 m** | **0%** |

The predicted breach assumed a **constant** 12 ft lane, half-width 1.829 m,
which would have put the chassis edge 0.116 m past the line at 1.54 m of
deviation. Against the **actual** variable lane it never crosses, because the
largest deviations happen where the course is **wide**, not where it is narrow;
the worst deviation and the narrowest section are at different places on the
lap. That is precisely the error a constant-width assumption produces, and it is
the same error, in the opposite direction, as the one that made the world and
the grid disagree.

So **"never left the course" stands**, and it is now measured against the
painted boundary rather than inferred from a centreline distance. Two honest
caveats: this is one run, and I cannot recompute the older 1.79 m run's
clearance because its pose log is gone.

**A failure worth recording.** On the first run the metric printed
`NOT MEASURED (No module named 'generate_igvc_world')`. The block is fed to
`python3` on **stdin**, where `__file__` is the literal string `<stdin>`, so
`dirname(abspath(__file__))` silently resolved to the working directory. It
failed in the right direction because the metric reports why it could not
measure instead of quietly omitting itself; a plain `except: pass` would have
printed nothing and looked like a clean run. The directory now arrives as
`argv[5]`.

### 10.7 P0-2 checked before building anything (C3)

**Answer: `/lane_map` IS produced with real content when only camera 0 is
flowing, so the override launch file is not needed for function. P0-2 drops to
"throttled WARN only", and the target stays `sim_cameras:=all`.**

Measured with the stock `lane_detection_config.yaml` (`num_cameras: 3`),
`sim_cameras:=front`, and the robot driving itself under the real autonomy stack
for 90 s:

| Topic | cells | occupied (>=50) | free (0) | unknown (-1) |
| --- | --- | --- | --- | --- |
| `/lane_map` | 102400 | **253** | 311 | 101836 |
| `/lane_costmap` | 160000 | **800** | 23600 | 135600 |

Watchdog warnings over the whole 90 s: **1**, i.e. a single startup warning, not
a persistent failure. No errors in the node log.

**I got this wrong twice before getting it right, and both errors are worth
recording because they are the project's own documented traps.**

*First wrong answer.* A static test (robot never commanded, 35 s window)
reported `/lane_map` as 102400 cells **all unknown**, and I read that as lane
detection producing nothing. It was an artifact of the test: the persistent map
accumulates hits above a threshold, so a stationary robot that has not swept any
lane into view has nothing to accumulate. "The topic is alive" is not evidence
perception works, and neither is "the grid is empty" evidence that it does not.

*Second wrong answer.* Chasing that, I compared one RGB header stamp against one
depth header stamp from two sequential `ros2 topic echo --once` calls, got
1.98 s, and nearly concluded the synchroniser could never fire against its
hardcoded 0.1 s slop. The two samples were taken about two seconds apart, so the
measurement recovered its own sampling interval and nothing else. The proper
test, subscribing to both simultaneously for 30 s, shows the opposite:

- RGB 397 messages, depth 396, over 26 s of simulated time, about 15 Hz each.
- Nearest-depth stamp gap: min **0.0000**, median **0.0000**, max **0.0660 s**.
- `ApproximateTimeSynchronizer(queue_size=5, slop=0.1)`, exactly as built at
  `lane_detection.py:88-89`, fired **396 times**.

So the RGB and depth streams are near-perfectly aligned and the QoS hypothesis
is refuted too: both publishers are **RELIABLE**, matching
`lane_detection.py:86-87`'s default subscription. The Phase 0 note that flagged
a possible QoS mismatch is now closed as a non-issue.

*A third, smaller trap.* My first probe printed "front camera SILENT" directly
underneath `average rate: 8.332`. That is `set -o pipefail` plus a `timeout` on a
long-running producer: `grep` succeeded, but `ros2 topic hz` exited 124 when the
timeout killed it, so the pipeline reported failure and the `||` branch fired.
`GAZEBO_SETUP.md` 10.5 documents exactly this and says "capture first, match
second". I walked into it anyway. It is also why the 6.7 and 9.0 Hz figures from
that probe are wrong: measured properly the cameras run at about 15 Hz.

---

## 11. Baseline branch: `main` versus `more_diverging_changes` (A4)

Read-only. The two are **siblings** off `upstream/main`, not ancestor and
descendant: `main..MDC` is 61 commits ending 2026-06-01, and `main` is 22 ahead.

| Dimension | `main` | `more_diverging_changes` | Changes a Phase 1 result? |
| --- | --- | --- | --- |
| Competition entrypoint | `auton_launch.sh` **absent** | `auton_launch.sh:3-9`, `docker start dazzling_easley` then `igvc_fused_drive.launch.py` | Yes, it defines the baseline |
| Lane node started | `lane_segmentation_node` (YOLOPv2) | **Same node**. `lane_detection_node` (Hough) is commented out on **both** | No |
| Model input size | `model_img_size: 384` | **640** | **Yes**, about 2.7x the inference cost |
| Weights path | Nitin's laptop, `/home/nitin-5090/...` | container-relative via `$YOLOPV2_WEIGHTS` | MDC is the portable one |
| Planner / controller | SmacPlanner2D / RegulatedPurePursuit | **identical** | No |
| `lookahead_dist` | 1.2 | **1.8** | **Yes** |
| `curvature_lookahead_dist` | 1.0 | **1.8** | **Yes** |
| Global `inflation_radius` | 0.75 | **1.00** | **Yes**, narrows the plannable corridor |
| Footprint, progress checker, goal checker | 0.70 m wide, 0.1 / 45.0, 0.5 / 0.5 | **identical** | No |
| Costmap plugins | `[obstacle_layer, lane_layer, inflation_layer]` | **`[lane_layer, inflation_layer]`**, obstacle_layer deleted | **Yes, the biggest single change** |
| Observation sources | 3 clouds + `/scan` in both costmaps | **none, no ObstacleLayer exists** | **Yes**, Nav2 gets zero live obstacle evidence |
| Where obstacles go instead | Nav2 costmaps | the **navigator**, via `lidar_obstacle_costmap_node` on `/lidar_obstacle_map` | **Yes**, avoidance moves out of Nav2 |
| collision_monitor StopZone | `enabled: true` | **`enabled: false`** | **Yes**, MDC's hard stop is off |
| Navigator | lane follow | + `/mission/state` yield gate, + **stuck-reverse recovery**, + path hysteresis, + heading limit | **Yes**, a Phase 1 run would reverse and can idle waiting on a node we do not launch |
| Navigator staleness gates | 5.0 / 5.0 / 0.6 / 0.25 | **2.0 / 0.8 / 2.0 / 0.4** | **Yes**, tighter gates will abort on a slow sim |
| **`wheel_radius`** | **0.2032** (flagged wrong) | **0.1016** | **P2-14, the wheel radius item, may already be answered on MDC.** Not RQ-02, which is GPU rendering and is unrelated |
| `sim:=true` argument | present, selects primitive colliders | **removed**; mesh colliders unconditional | **Yes, MDC segfaults dartsim** |
| Mesh paths | `package://...` | **`file:///home/nitin-5090/...`** | **Yes, the description loads nowhere** |
| Sensor poses | lidar `0.225 0 0.46`, ZED yaw +/-95 deg | lidar **`0.345 0 0.52`**, ZED yaw **+/-45 deg**, **4th rear camera** | **Yes** |
| `localization_node` | launched | **deleted**, replaced by static identity `map->odom` | **Yes** |
| The entire simulator | 11 scripts, world SDF, Gazebo xacro, bridge YAML, odom shim, both Gazebo launch files | **all absent** | **Yes** |

**It is a port, and it runs in the simulator's direction.** You cannot check out
MDC and simulate: it removes `sim:=true`, restores the four zero-normal mesh
colliders that segfault dartsim, and rewrites every mesh URI to an absolute path
on a machine nobody has. You would carry `main`'s Gazebo layer onto MDC's stack,
file by file.

**Recommendation: do not switch baselines now**, which is also the instruction.
But note the two facts that matter for planning: MDC already sets
`wheel_radius: 0.1016`, and MDC's Nav2 has **no obstacle layer at all**, so
adopting its configs would change what Phase 1 is testing rather than tune it.

---

## 12. Questions for Aidan

Revised. The first replaces my earlier questions 1 and 5, which assumed the 2026
AGX is reachable.

1. **Where is the 2026 AGX now, and was its disk reflashed?** If the disk
   survives, two commands settle the whole topology question:
   `docker ps -a` looking for **`dazzling_easley`**, and
   `systemctl status devmem-init`. That container name is hardcoded in all five
   of Nitin's competition scripts.
2. You said "the Dockerfile on `main`" ran at competition. Could the **image**
   have been built from `main` while the **code** bind-mounted into it was
   `more_diverging_changes`? `main` got one unrelated commit during the event;
   MDC got 54. That reconciles your answer with the history rather than
   contradicting it.
3. **Which lens is fitted to the three ZED X units?** Or better, does anyone
   have a competition bag with a `camera_info` in it? One message settles the
   simulated camera's field of view, which currently has an unknown sign of
   error.
4. Is `wheel_radius: 0.2032` correct on the real robot, or is it the diameter?
   Note `more_diverging_changes` already sets **0.1016**, so this may already be
   answered by the 2026 team.
5. RQ-06: `gz_ros2_control` in-process, or a `GazeboDriveHardware` topic bridge?
6. RQ-11: should barrels tip, or stay static?
7. Did the 2026 team record a course origin (lat/lon)? If not, I will pick an
   arbitrary datum for `<spherical_coordinates>` and label it as arbitrary.

**Two things that affect the real robot, flagged only, nothing changed:**

- **collision_monitor's point-cloud sources are all `enabled: false`** in the
  shipped real config (`nav2_lane_follow_config.yaml:189, 197, 205`), and
  `observation_sources` lists only `front_pointcloud`. Its stop and slow polygons
  therefore have no live input and can never trip on the real robot.
- **`lane_layer`'s `lethal_cost_threshold` is 90** (`:325`, `:414`) while
  `gt_nav_bridge_node.py:274` writes lane cells as exactly **100**. So lane paint
  becomes `LETHAL_OBSTACLE` (254), not the high-cost 253 the comment at `:315-316`
  promises, and gets inflated by `inflation_radius: 0.75`. The comment claims the
  threshold is "ABOVE 100"; it is 90. That is the failure mode the same comment
  says forced `use_collision_detection: false`.

---

## 13. Two process notes

**The branch instruction arrived with both options still in it.** A3 offered
"fast-forward `main` to `docs-trim`" and "leave `docs-trim` untouched" and the
choice was not made before sending. Both agree on working in `gazebo-phase1` and
never on `docs-trim`, so I took the reversible option and branched from `main` as
it stands. If you meant to fast-forward, say so and I will rebase.

**The docs instruction presumes a layout that only exists on `docs-trim`.** A2
says not to edit archived files, and to put live status in `docs/STATUS.md`. On
`main` there is **no `docs/archive/` and no `docs/STATUS.md`**:
`GAZEBO_TODO.md`, `DOCKER_CHANGES.md` and `RQ03_AUDIT.md` are all still live at
`docs/` top level. So on this branch there is nothing archived to avoid editing,
and creating `docs/STATUS.md` here would collide with the one on `docs-trim` at
merge.

What I did: kept the entire Phase 1 record in this new file, which merges cleanly
either way, and made no other documentation edits. **The to-do list and the live
status still need updating, and I have deliberately not done it** until the
branch question in §13 is resolved, because the right destination depends on the
answer. That is the one deliverable from your list I am holding back, and it is
one edit either way.

---

## 13A. Corrections to Phase 1

Raised on review, 2026-09-17. Two overturned my numbers, one found real
duplication, one was a mislabel.

**D-1. The clearance metric ignored yaw, and yaw eats two thirds of the margin.
Confirmed and corrected.** A fixed half-width is the robot's lateral extent only
when it is aligned with the lane. Skewed, a corner leads, and the reach
perpendicular to the lane is `(W/2)|cos t| + (L/2)|sin t|`. Taking the length
from the URDF bounding box rather than the design report, the chassis is
**0.810 x 0.970 m**, so the reach runs from 0.405 m aligned to
`hypot(0.405, 0.485) = 0.632 m` at the worst angle, 50.1 degrees.

This is not a hypothetical. Measured across a full run the robot sits at a
**mean 20.7 degrees** to the lane and peaks at **65.3 degrees**, because the
navigator is continuously re-aiming. Recomputed with the rotated footprint, on
three fresh runs:

| Run | Max centreline deviation | Worst clearance, chassis | Worst clearance, Nav2 | Over the line |
| --- | --- | --- | --- | --- |
| 4 | 2.04 m | **+0.093 m** | +0.135 m | 0.0% |
| 5 | 1.95 m | **+0.087 m** | +0.127 m | 0.0% |
| 6 | 1.98 m | **+0.078 m** | +0.119 m | 0.0% |

So the conclusion survives, and I will say it plainly: **the robot never put a
corner over the paint in any run.** But the margin is **8 to 9 cm**, not the
25 cm I reported yesterday. That is a much less comfortable number and it is the
right one.

**D-2. 1.92 Hz was too slow, and the error was 2.4x.** Raised to 10 Hz
(`POSE_HZ`), achieving about 15 Hz in practice. Re-scoring one 831-sample run at
decreasing rates isolates the effect:

| Effective rate | Worst clearance | Max centreline deviation |
| --- | --- | --- |
| 15.8 Hz | **+0.093 m** | 2.04 m |
| 7.9 Hz | +0.116 m | 2.04 m |
| 4.0 Hz | +0.153 m | 2.04 m |
| 2.0 Hz | **+0.227 m** | 2.04 m |

The two quantities behave differently, which is itself the finding: **the worst
clearance is a brief event and the worst deviation is not**, so the old rate
overstated the margin by 2.4x while leaving the deviation untouched. Note the
downsampling is a weaker test than a genuine low-rate run, because it starts
from a dense log and can retain the peak sample; treat 2.4x as a lower bound on
the error.

**D-3. A regression baseline that is now visibly a coin flip, and I am not
touching it.** At honest sampling the `IN LANE` gate produced **2.04 (FAIL),
1.95 (PASS), 1.98 (PASS)** against its flat 2.00 m tolerance. Earlier runs at
2 Hz reported 1.53, 1.54 and 1.93. One run in three now fails.

The behaviour did not regress: 47 to 51 m driven under its own control every
time, on the slab throughout, and 0.0% of samples over the paint in all three.
What changed is that the measurement got honest. **The gate is measuring the
wrong quantity** (a constant tolerance against a lane whose width varies by a
factor of two) and it sits right on its own threshold. Per instruction I have
**not** changed the criterion, and I have not raised `TOL` to make it green,
which would be the wrong fix twice over. **This needs your decision**, and it is
the one thing currently making `autonomy_check.sh` unreliable as a regression
gate.

**D-4. The width profile is now imported, not copied.** `constants.py` in the
track-generator submodule is a pure constants module with no imports or side
effects, so `generate_igvc_world.py` imports it directly and a missing submodule
is a hard error rather than a guessed fallback. Verified: the imported values
regenerate a **byte-identical** world, so the hand-mirrored numbers were right
and nothing about the course changes. No consistency check is needed because
there is no longer a second definition.

**D-5. The "RQ-02" label in §11 was wrong.** RQ-02 is GPU rendering. The wheel
radius item is **P2-14**. Fixed in place.

**D-6. The sub-10-ft corridor is definitional, not systematic.** `track.png`'s
narrowest corridor is 0.282 m under the ten-foot minimum, and that is exactly
two painted line widths: the 10-to-20 ft range is the **outer** track width
including the paint, so the drivable corridor between inner edges is that minus
`2 x 5 px = 0.2822 m`. `int(10 * 10.8)` is 108 exactly, so truncation
contributes **nothing** at the minimum. The residual 0.023 m between nominal
(2.766 m) and measured (2.743 m) is sub-pixel rasterisation of a stamped disc.

---

## 13B. The three P0 items

### P0-1: cross-container DDS, a partial pass with an open defect

The scope narrowed first: `lane_detection` (Hough) needs no torch and already
runs inside the Gazebo image, so only `lane_segmentation_node` (YOLOPv2) needs
the Humble container. The gate was run on the topics that node consumes,
counting messages in both containers **simultaneously** for 30 s.

| Topic | Jazzy (same container as the sim) | Humble (across the boundary) |
| --- | --- | --- |
| rgb image | 354 | 356 |
| depth | 353 | 355 |
| camera_info | 357 | 357 |
| zed odom | 691 | 691 |
| /odom | 691 | 691 |
| /clock | 17188 | 17011 |
| /tf | 1161 | 1159 |

Within 1%, comfortably inside the 10% criterion. Payloads are right too:
691200 B = 640x360x3 for RGB, 921600 B = 640x360x4 for 32FC1 depth, and
`/tf_static` delivers its one latched message with 24 transforms once the
subscriber uses `TRANSIENT_LOCAL` (my first probe used sensor-data QoS and read
zero, which was my bug, not a finding).

**THE OPEN DEFECT.** The Humble container printed, verbatim and exactly three
times per process:

```text
sequence size exceeds remaining buffer
```

What I established before stopping:

- It is **not topic-specific**. It appears three times whichever single topic is
  subscribed: rgb, depth, camera_info, odom, clock, tf and tf_static each gave
  the same three lines.
- It is **not data-related**. A Humble node subscribing to **nothing at all**,
  with the Jazzy graph up, still prints all three.
- It is **directional**. A Jazzy node subscribing to nothing prints **none**.
- Every payload that arrives is intact and the counts match.

So the working hypothesis is a **discovery-phase** parse failure: Humble's Fast
DDS cannot read something Jazzy's participants advertise. **That is a
hypothesis, not a conclusion.** I did not capture the failing packet, did not
identify the field, and did not rule out that some low-rate message is being
dropped silently in a way 30 s of counting would not reveal.

**This gate is a partial pass, not a pass.** Message counts staying intact while
a deserializer complains is precisely the silent-partial-failure class that has
bitten this project twice. It should be closed properly before YOLOPv2 results
from the Humble container are trusted.

### P0-3: point-cloud orientation, answered, and it is the bad answer

The cloud is emitted in the **body** convention (x forward) and stamped with the
**optical** frame. `gz_frame_id` sets a header string; it does not rotate data.

| Run | Normal axis | Offset on that axis | Angle to +Z | Inliers |
| --- | --- | --- | --- | --- |
| Through the cloud's own stamped frame | **X** | **-0.0994 m** | **89.78 deg** | 100% |
| Forced through the body frame (control) | **Z** | **-0.2311 m** | **0.39 deg** | 100% |

Predicted beforehand from the URDF geometry: **-0.096 m** and **-0.229 m**.
Measured: -0.0994 and -0.2311. **Agreement to 3.4 mm and 2.1 mm.** That is the
part worth keeping: the model of the bug predicted the numbers before the
measurement, so the fix is a known quantity rather than a guess.

The extents make it plain. Through the stamped frame the whole cloud collapses
into a 12 cm slab, x from -0.22 to -0.10, which is the floor stood on its end.
Through the body frame it is a ground plane running x from 1.14 to 16.22 m at
z = -0.23.

**Infinity filtering: the cleanup landed, and the numbers above are POST
cleanup.** The first run reported extents of `inf` and `nan` and only 45%
inliers, because `skip_nans` does not drop infinities and the bridge passes
exact ray-cast range with `+inf` past the far clip. After filtering explicitly,
127022 of 230400 points are dropped as non-finite, 103378 remain, and both fits
reach 100% inliers. The verdict was the same before and after; the numbers are
only trustworthy after.

The check is committed as `scripts/gazebo/cloud_frame_check.py` and it names
which of four mistakes was made rather than just failing.

### P0-3 consequence, which the next session must not miss

`nav2_lane_follow_config.yaml:251-258` lists `obstacle_layer` **enabled**, with
`observation_sources: "front_pointcloud left_pointcloud right_pointcloud
lidar_scan"`, in both costmaps. But `gazebo_nav_test_nav2_overrides.yaml:36-44`
replaces `plugins` with `["lane_layer", "inflation_layer"]` in both costmaps,
which stops `obstacle_layer` being instantiated at all.

**So the answer is the second one: the clouds never reach `obstacle_layer`, and
`autonomy_check.sh` has been passing with no live obstacle evidence whatsoever.**
Every obstacle the robot has ever avoided came from `track_points.json` through
`gt_nav_bridge_node`, which `GAZEBO_SETUP.md` 10.3 already says plainly.

The consequence for the next session is the dangerous half: **fixing the cloud
frame and re-enabling `obstacle_layer` connects a consumer that has never been
connected.** That is not a bug fix, it is a new system under test. Expect
autonomy behaviour to change, re-baseline all three gates afterwards, and do not
attribute the change to the frame fix alone.

### P0-4: sim-side lane evaluator, built and run, first numbers in hand

`scripts/gazebo/lane_eval_sim.py` scores a detector against the lane **lines**
generated from the same `track_points.json` and the same width profile the world
is painted from, so reference and stimulus are the same geometry by
construction. It reports cell IoU plus two distance-tolerant scores at a stated
tolerance, because a painted line is under one cell wide and IoU swings on a
one-cell offset.

Measured, robot driving itself for 100 s, front camera only, tolerance 0.25 m:

| | `lane_detection` (Hough), own grid | control: the ground-truth bridge's grid |
| --- | --- | --- |
| grid | 320x320 @ 0.125 m | 800x800 @ 0.100 m |
| occupied cells | 328 | 2946 |
| cell IoU | 0.0434 | 0.2314 |
| hit rate @ 0.25 m | **0.491** | 0.676 |
| coverage, whole window | 0.115 | 0.996 |
| coverage, observed region | **0.464** | 0.996 |
| predicted to paint, median | 0.255 m | 0.096 m |

**These are the first non-zero lane metrics this project has produced.** For
scale, design report figure 7 reported IoU 0.083 with 96.8% of frames dropped,
and it was measuring a units mismatch.

Read them against the control, not against 1.0. The control is a grid derived
from the same JSON as the reference, and it still only reaches a hit rate of
0.676, because `gt_nav_bridge_node` stamps a lethal rim near but not exactly on
the painted line. **0.676 is the practical ceiling of this metric, so the Hough
detector's 0.491 is about 73% of achievable.** Coverage over the whole window
(0.115) is unfair to a camera that only observed 636 cells in one lap; restricted
to reference paint within 1.5 m of an observed cell it is **0.464**.

**A measurement error I made and caught.** The first run scored 0.2314 IoU and
0.996 coverage and I nearly reported it as the detector's. It was not.
`lane_detection.py` hardcodes `/lane_map`, and `gazebo_nav_test.launch.py` also
starts `gt_nav_bridge_node`, which publishes the same name. The grid I scored was
800x800 at 0.100 m, which is the ground-truth bridge's geometry, not
`lane_detection`'s 320x320 at 0.125 m from its own config. **I was scoring the
ground truth against itself.** Caught by checking the grid geometry against the
config rather than trusting the topic name. The rerun remaps to
`/hough/lane_map`.

**Not done:** `lane_segmentation` (YOLOPv2) is unscored. It needs weights that
are not on disk (`models/` does not exist; `fetch_yolopv2_weights.sh` downloads
them and verifies no checksum) and it must run in the Humble container, which
depends on P0-1 being closed properly.

---

## 14. Commits prepared, and the push command

Branch **`gazebo-phase1`**, created from `main` (`8a38fbc`). Five commits, one
topic each. Nothing has been pushed.

| Hash | Message |
| --- | --- |
| `d005781` | Make render_check.sh work on a clean container |
| `8ee7e0e` | Reject an unrecognised sim_cameras instead of building a robot with none |
| `8e773b2` | Let the autonomy path set the camera resolution |
| `97a9704` | Paint the course the planner actually plans in, not a constant 12 ft lane |
| `961b99f` | Report how close the robot came to the painted line, not just to the centreline |

Plus this report as a sixth commit.

```bash
cd "/c/IGVC 2027/IGVC_robot_2027"
git push -u origin gazebo-phase1
```

`main` and `docs-trim` are untouched. Nothing in this branch depends on which
way you resolve the `docs-trim` question, except the one documentation update I
am holding back (see §13).

### Files changed

| File | Why it is allowed to change |
| --- | --- |
| `scripts/gazebo/render_check.sh` | `scripts/gazebo/` is sim-side |
| `scripts/gazebo/autonomy_check.sh` | same |
| `scripts/gazebo/generate_igvc_world.py` | the world generator, explicitly sim-side |
| `src/igvc_test_description/worlds/igvc_course.sdf` | generated output of the above |
| `src/igvc_test_bringup/launch/gazebo_sim.launch.py` | a sim launch file |
| `src/igvc_test_bringup/launch/gazebo_nav_test.launch.py` | a sim launch file |
| `src/igvc_test_description/urdf/gazebo/gazebo_sim.urdf.xacro` | the Gazebo-only description, included under `sim:=true` |

**Nothing on the forbidden list was touched.** No edit to
`src/igvc_lane_detection/`, `nav2_lane_follow_config.yaml` or its siblings,
`controllers.yaml`, `lane_detection_config.yaml`, the real robot URDF or its
meshes, or anything on the `CanInterface`/ODrive path. Confirmed by the diff.

### Regression status

| Gate | Result |
| --- | --- |
| `render_check.sh` | PASS, exit 0, now from a clean shell |
| `bringup_smoke_test.sh` | **18 passed, 0 failed**, exit 0, ratio 0.9990 |
| `autonomy_check.sh` | PASS, exit 0 (see §10.6) |

### What is left, in order

1. The documentation update being held on the branch question (§13).
2. P0-1 and P0-3: the cross-container DDS gate, then point-cloud orientation.
3. P0-4, the lane evaluator, now that §9 C-2 has established what it must
   compare and why the existing one returns zero.
4. P1 and P2 as planned in §6, with the FollowPath item reframed by §9 C-3:
   instrument the abort count first, because the number everyone has been
   quoting is a saturating gauge.

---

## 16. Merging into `main`, and getting it clonable for the sub-team

**Written 2026-09-17, after the Phase 1 work above and before the 6 to 8 pm
software session.** The goal of this pass was narrow and it outranked the
parity work: make `main` something a teammate can clone on an untested Windows
laptop and end up with a robot driving the course.

### 16.1 What merged, what did not, and the gate numbers on the merge result

**`gazebo-phase1` merged into `main` as a fast-forward.** Confirmed first that
`main` had not moved (`8a38fbc`, identical to `origin/main`) and that
`git merge-base --is-ancestor main gazebo-phase1` held, so the merge result is
byte-identical to the branch tip `a7cf7b9` and no merge commit exists. Nothing
else was merged: `docs-trim` was not touched, and its five unpushed commits of
unknown provenance are still sitting there untouched.

Confirmed by diff that **nothing on the forbidden list is in the merge**: no
`src/igvc_lane_detection/`, no `nav2_lane_follow_config.yaml` or siblings, no
`controllers.yaml`, no `lane_detection_config.yaml`, no real robot URDF or
meshes, nothing on the `CanInterface`/ODrive path.

**All three gates pass on `main` itself**, run in `igvc_gazebo` against the
merge result:

| Gate | Result | Numbers |
| --- | --- | --- |
| `render_check.sh` | **PASS**, exit 0 | `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, Mesa 25.2.8, camera and gpu_lidar both `PUBLISHING`. Run from a shell with nothing sourced |
| `bringup_smoke_test.sh` | **PASS**, exit 0 | **18 passed, 0 failed.** Odometry 7.073 m against Gazebo ground truth 7.081 m, heading disagreement **0.13 deg**, distance ratio **0.9988** |
| `autonomy_check.sh` | **PASS**, exit 0 | 893 samples at 9.5 Hz over 93.7 s. **83.0 m driven**, span 29.3 m, centreline deviation mean 0.67 m / **max 1.86 m**, yaw to lane mean 15.9 deg / max 48.0 deg, worst clearance **+0.040 m** (chassis) and +0.081 m (Nav2 footprint), **0.0% of samples over the paint**, on the slab throughout, `aborts=2`. MOVED, PROGRESS, IN LANE and ON THE SLAB all PASS |

**The `IN LANE` coin flip did not bite this time, and that is luck rather than
a result.** 1.86 m against a flat 2.00 m tolerance is a pass, but section 13A
D-3 records 2.04 (FAIL), 1.95 (PASS), 1.98 (PASS) on three earlier runs. The
gate is still measuring the wrong quantity and is still one run in three from
failing. It was not changed, and `TOL` was not raised. Note also that this
run's worst clearance, **+0.040 m**, is tighter than the +0.078 to +0.093 m of
runs 4 to 6, which is the tightest margin yet recorded.

**The gate results still stand across every commit since**, and that is
checked rather than assumed: `git diff --name-only a7cf7b9..HEAD` returns only
documentation, the new `scripts/gazebo/bootstrap.sh` and
`scripts/setup-windows.ps1`. Nothing under `src/`, `config/`, `docker/`, the
compose files, or any of the gate scripts themselves changed after the gates
ran. `render_check.sh` and `bringup_smoke_test.sh` were then re-run anyway,
from a clean clone, in 16.3.

### 16.2 The judgement call on P0-3, made explicitly

**The P0-3 point-cloud frame fix does not exist on the branch, so the question
answered itself.** The branch carries `scripts/gazebo/cloud_frame_check.py`,
which is the diagnostic that proved the rotation, and no fix: no republisher,
no bridge YAML change, no costmap override change. The diff is the evidence.

**It would not have merged tonight even if it had existed**, and the reason is
worth stating because it is the freeze rule doing its job. The fix only has a
purpose if `obstacle_layer` is enabled, and
`gazebo_nav_test_nav2_overrides.yaml:36-44` replaces `plugins` with
`["lane_layer", "inflation_layer"]` in both costmaps, so that layer has never
been instantiated and the clouds have never reached it. Enabling it connects a
Nav2 consumer that has never once been connected. That is a new system under
test, not a bug fix, it would need all three gates re-baselined afterwards,
and it cannot clear that bar before 6 pm. It stays out, and `GAZEBO_TODO.md`
records it as a fix with the re-baseline requirement attached.

### 16.3 Verification from a clean clone

Not from the working copy, and with nothing sourced in advance. The clone was
taken from the local repository rather than from GitHub, because **the push
has not happened**, so one thing this does not verify is the remote itself.
Everything else is faithful: submodules were fetched from their real GitHub
URLs, the compose project was isolated with `COMPOSE_PROJECT_NAME=freshtest`
so it could not reuse the working copy's named volumes, the image was **removed
from the daemon first** so it could not reuse a warm one, and the whole thing
ran from a real WSL2 Ubuntu shell rather than Git Bash.

| # | Step | Result | Elapsed | What a teammate sees |
| --- | --- | --- | --- | --- |
| 1 | `git clone --recurse-submodules` | **PASS** | **17.4 s** | nine `Submodule path ... checked out` lines |
| 2 | `.sh` line endings and shebangs | **PASS** | under 1 s | 12 of 12 files LF, every shebang `#!/usr/bin/env bash` with no trailing CR |
| 3 | bootstrap step 1, sanity | **PASS** | under 1 s | `PASS running from a WSL2 shell`, `PASS free disk: wanted 12 GB, found 107 GB` |
| 4 | bootstrap step 2, submodules | **PASS** | 2 s | `PASS all 9 submodules populated`, `PASS track data and zed_description are on disk` |
| 5 | bootstrap step 3, image from the tar | **PASS** | see below | `loading /mnt/c/.../igvc-gazebo-jazzy.tar (1.2G). No pull, no build.` then `Loaded image: igvc-gazebo-jazzy:latest` |
| 6 | bootstrap step 4, container | **PASS** | 3 s | `Volume freshtest_igvc_gazebo_log Created`, `Container igvc_gazebo Started` |
| 7 | bootstrap step 5, colcon | **PASS** | about 15 s | `PASS four packages built` |
| 8 | bootstrap step 6, `render_check.sh` | **PASS** | about 25 s | `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, `RESULT: HARDWARE RENDERING (NVIDIA)` |
| 9 | bootstrap step 7, `bringup_smoke_test.sh` | **PASS** | about 80 s | `topic checks passed: 18 failed: 0`, `DRIVE TEST : PASS robot moved 7.331 m`, `FRAME TEST : PASS`, ratio **0.9989**, heading **0.13 deg** |
| | **whole bootstrap** | **exit 0** | **201 s** | `Bootstrap complete. The simulator is verified on this machine.` |

**The written quickstart needed no corrections during the run**, which is the
result that matters, but only because the clone-breaking defects had already
been found by reading and were fixed before the run rather than during it. The
defects themselves are listed in 16.5; the biggest was that the documented
clone command had no `--recurse-submodules`.

**Timings, and an honest caveat on one of them.** The offline path was measured
separately and end to end:

| Measurement | Result |
| --- | --- |
| `docker save igvc-gazebo-jazzy:latest` | **14.6 s**, producing **1,201,505,280 bytes (1.20 GB)** |
| `docker load -i`, image **genuinely absent** | **1 m 17.0 s** |
| the image unpacked, after the container has run | 5.57 GB |

The 77 s figure is a true cold load: `docker image rm` was run first and
`docker image ls` confirmed the image gone. **The load inside the 201 s
bootstrap run was faster than that**, because the layer blobs had been in the
content store moments earlier, so the honest end-to-end figure for a teammate
whose machine has never seen this image is **201 s plus up to about a minute**.
That is why the quickstart says "about 10 minutes" rather than quoting 201 s.

**A finding worth more than the timings: the tar is 1.20 GB and the image is
5.57 GB, and both numbers are right.** The containerd image store unpacks
layers lazily. Immediately after `docker load`, `docker image ls` reported
**1.2 GB**; after the container had run once, the same image reported
**5.57 GB**. Observed today on the same image. Plan disk against 5.57 GB, and
expect a teammate to quote the other number in good faith.

**Disk high-water mark, and a trap that cost 14 GB.** Free space on C: went
from **120.7 GB** at the start of the session to a low of **105.1 GB**, and
after removing the test clone, the tar and the `freshtest` volumes it came back
to only **106.5 GB**. So roughly **14 GB was consumed and not recovered.** The
cause is the documented one: `docker_data.vhdx` never shrinks, is not sparse,
and Windows 11 Home has no Hyper-V so `Optimize-VHD` is unavailable. Removing
an image and reloading it allocates new blocks in the VHDX while the old ones
stay allocated. **Do not tell teammates to remove and reload images casually**,
and be aware that the 40 GB floor gets closer every time someone does.

The test clone, the tar and the `freshtest` volumes were all removed, and the
working `igvc_gazebo` container was brought back up against the real checkout.

### 16.4 Everything changed in this pass, one line each

Scripts:

| File | Change |
| --- | --- |
| `scripts/gazebo/bootstrap.sh` | **New.** One command from a fresh clone to a verified simulator: submodules, image or tar, container, colcon, `render_check.sh`, `bringup_smoke_test.sh`, PASS or FAIL per step, stops at the first failure |
| `scripts/setup-windows.ps1` | Builds nothing now; was building a 14.1 GB image the simulator does not use. Measured disk threshold printing both wanted and found. Windows 10's three build thresholds separated. Weights made opt-in. One real bug fixed, below |

Documents:

| File | Change |
| --- | --- |
| `docs/GAZEBO_QUICKSTART.md` | Rewritten end to end. `--recurse-submodules`, the offline USB path, derived disk numbers, four packages not three, today's reference numbers, the `IN LANE` coin flip, and a section 10 that says what does not work without hedging |
| `docs/GAZEBO_SETUP.md` | 10.2a and 10.4 retract the lane width, the abort count and the corridor arithmetic. New 10.7 to 10.10 record the Phase 1 changes. 10.6's point-cloud item moves from unverified to measured |
| `docs/GAZEBO_TODO.md` | The update held pending the branch question. P0-1, P0-3 and P0-4 results, the `IN LANE` decision as a TEAM item, the corrected clearance margin, and one false retraction corrected |
| `docs/DOCKER_CHANGES.md` | New section 15: the images and which one you need, the two sizes 4.6x apart, the measured offline path, volumes, every environment variable, the `FASTDDS_DEFAULT_PROFILES_FILE` asymmetry, amended outstanding list |
| `README.md` | Opens with what the project is, that it is the team repo as of September 2026, three commands to a driving robot, and what works and does not with numbers |
| `docs/IGVC_2027_Gazebo_Setup_Guide.docx` | Regenerated from the quickstart so the Word copy cannot disagree with the Markdown |
| `SOFTWARE_SESSION_2026-09-17.md` | **New**, and **no longer in this repo**, see 16.8. Pre-flight, a timed two-hour plan, five likeliest failures, the no-GPU fallback, the demo order, eight known-not-working items to read aloud, four questions for Aidan |
| `CLAUDE.md` (in-repo) | Said "ROS 2 **Humble**" at the top. Corrected, but the file is excluded in `.git/info/exclude` and has never been tracked, so the change is local-only. See C-19 |

**One bug found by running a script rather than reading it.** The new
`-Weights` switch on `setup-windows.ps1` collided with its own local
`$weights` variable, because PowerShell pre-declares parameter variables and is
case-insensitive, so assigning a string threw
`ConvertToFinalInvalidCastException` at the end of an otherwise clean run.
Renamed to `$weightsPath`. This is the fourth time on this project that running
the thing found something reading it did not.

**Both new failure paths in `bootstrap.sh` were shown to fail**, per the rule
that every new check must be demonstrated on a deliberately broken input. From
the wrong directory it stops at step 1 naming the directory it wanted; with a
nonexistent `IMAGE_TAR` it stops at step 3 with the `/mnt/d` hint. Both exit 1.

### 16.5 Documentation that was wrong before this pass

Listed as corrections because a teammate would have hit each one.

**C-10. The documented clone command had no `--recurse-submodules`, and it
breaks the build.** `GAZEBO_QUICKSTART.md` section 2 said
`git clone https://github.com/...`. There are nine submodules and two are
load-bearing: `zed_description` is one of the four packages colcon builds **by
name**, so the workspace build fails outright on a package the user never
touched, and `IGVC_track_generator` holds `track_points.json` and `track.png`,
which every navigation node and the world generator read. **This is the single
most likely way tonight could have failed for everyone at once**, and it would
have presented as a confusing colcon error. Fixed, with the expected count, how
to check, and how to repair an existing bad clone; `bootstrap.sh` also refuses
to continue if any submodule is empty and checks two specific files exist
rather than trusting the submodule pointer.

**C-11. `GAZEBO_TODO.md` claimed a retraction that had not happened.** It said
"`GAZEBO_SETUP.md` 10.4 already retracts its own 0.87 m gap". Section 10.4
still asserted both the 0.87 m gap and the 0.70 m robot, at lines 1106 to 1107,
until today. The Phase 0 text in section 4 above repeated the same claim. Both
are corrected, and the new 10.4 says explicitly that the forward reference was
false, because a pointer to a retraction that does not exist is worse than no
pointer.

**C-12. `GAZEBO_SETUP.md` 10.2 rested on "the lane is about 3 m wide".** That
was the load-bearing assumption under the whole section's reasoning about
whether the robot stayed in the lane. The corridor Nav2 actually plans in
measures 2.766 to 6.068 m.

**C-13. The quickstart's autonomy reference numbers came from a 2 Hz pose
log.** It quoted 122.9 m driven and 1.79 m maximum deviation. Downsampling a
dense run shows that rate reports a worst clearance of +0.227 m where honest
sampling reports +0.093 m, so those numbers understated the risk by at least
2.4x. Replaced with today's, and `GAZEBO_TODO.md`'s milestone line replaced
too.

**C-14. The quickstart made `render_check.sh` the first command on a new
machine and told you to source ROS by hand, while `GAZEBO_TODO.md` listed the
script under "Done, and verified".** It failed on a clean container until
`d005781`. Anyone following the written setup would have seen `RESULT: UNKNOWN`
and read it as a broken GPU. Already retracted in Phase 1; recorded here again
because it is a documentation failure and not only a script failure.

**C-15. `setup-windows.ps1` ended by building a 14.1 GB image the simulator
does not use, then told the user to open it with `docker compose run`.** On
shared campus wifi with several people at once, the first is the entire
meeting. The second is the documented trap that once produced six simultaneous
`gz sim` processes. Both removed.

**C-16. Its disk threshold was a flat 30 GB, neither measured nor right.** The
Gazebo-only path measures 7.4 GB. The threshold is now 12 GB for the offline
path, derived line by line in the quickstart, and the check prints both the
number it wants and the number it found. The 25 GB build figure is labelled a
margin rather than a measurement, because 21.86 GB of this machine's 29.06 GB
of build cache is shared across four images and no single image's share is
cleanly attributable.

**C-17. The colcon build is four packages and the documentation said three.**
Both the quickstart and the comment at the top of `start_sim.sh` said three.
The code selects four and the code is right; `zed_description` is the one
people leave out. The quickstart now says four and flags the stale comment.

**C-18. There was no offline path documented at all**, despite the image being
carried on a USB drive. Now measured and written down at both ends, including
how to spot a truncated tar and the exFAT limit.

**C-19. The in-repo `CLAUDE.md` said "ROS 2 Humble" in its second line, and
it turns out nobody but Liam can read it anyway.** The simulator is Jazzy and
has been since `fac55cb`. The line was corrected, **but the correction is
local-only and does not reach the team**: `CLAUDE.md` is excluded in
`.git/info/exclude`, which is a per-clone exclusion rather than `.gitignore`,
and the file **has never been tracked**. So a teammate cloning `main` gets no
in-repo `CLAUDE.md` at all, and the wrong distro line was never distributed
either. `git add` was refused and the exclusion was **not** overridden, because
someone chose it deliberately and reversing that is a repository-policy
decision rather than a documentation fix. **This needs a decision:** either
commit the file so the orientation it provides is shared, or delete it so
nobody edits a file that goes nowhere. Same question as the session-root
documents, which also exist only on one disk.

**C-20. `docs/GAZEBO_QUICKSTART.md` claimed "Verified 2026-09-15" while
describing behaviour that was fixed on 2026-09-17.** Dates and the machine
specification are now stated together at the top, with the explicit warning
that the numbers are x86 laptop numbers and not AGX numbers.

Also, 111 em dashes were replaced with commas across `GAZEBO_SETUP.md`,
`GAZEBO_TODO.md`, `DOCKER_CHANGES.md`, `README.md` and `CLAUDE.md`, per the
project convention. Mechanical, no change of meaning.

### 16.6 What this pass deliberately did not do

- **No perception work.** P0-1's deserialization defect, P0-3's fix and P0-4's
  YOLOPv2 scoring are all untouched, and P1 and P2 were not started.
- **The `IN LANE` gate was not changed**, and `TOL` was not raised. It needs a
  team decision and it is a one-line change once made.
- **`docs-trim` was not touched.**
- **Nothing was pushed.** See 16.7.
- **No image was rebuilt**, so the 40 GB disk floor was never approached,
  although 16.3 records 14 GB lost to the VHDX anyway.

### 16.7 The push commands, in order

PowerShell, which is where these actually get run:

```powershell
cd "C:\IGVC 2027\IGVC_robot_2027"
git push origin main
```

Git Bash, if that is the shell in hand:

```bash
cd "/c/IGVC 2027/IGVC_robot_2027"
git push origin main
```

Optionally, to put the branch pointer on GitHub as well. It is already merged
into `main` as a fast-forward, so this is for provenance only and is not needed
for anyone to clone:

```powershell
git push origin gazebo-phase1
```

**Do not hand a `/c/...` path to PowerShell.** It resolves it as a relative
path, tries `C:\c\IGVC 2027\...`, fails, and the `git push` on the next line
then runs in whatever directory you were already in and reports "not a git
repository". PowerShell 5.1 has no `&&`, which is why these stay two lines.

Check the exact count yourself with `git status -sb`; every attempt to
write it down here changed it, because writing it down is another commit. Nothing
in tonight's session works until this push lands, because the whole session is
people cloning it. Confirm it by opening the repository on GitHub and checking
that `scripts/gazebo/bootstrap.sh` appears in the web view.

### 16.8 Two working documents moved off the public repo

**Requested by Liam after the push.** `docs/SESSION_HANDOFF.md` and
`docs/runbooks/` are internal working notes: they name individuals, discuss
whose machine is likely to fail and how, carry meeting logistics, and in the
handoff's case carry a list of this project's own claims that are known false.
That is useful to us and it is not public-repo material.

Both are now kept at `C:\IGVC 2027\local-notes\`, alongside the other
local-only documents, and verified byte-identical to what was removed before
the removal happened. `.gitignore` carries both paths so nobody, human or
agent, re-adds them by accident.

**The limit of this, stated plainly: removing them from the tip does not
remove them from the published history.** They were pushed before the request
arrived, so `docs/SESSION_HANDOFF.md` remains readable on the public repo at
`2343dad`, `781fc68` and `a7cf7b9`, and the runbook at `67a9bac`, to anyone who
clones or who has the commit URL. **This commit reduces visibility, not
exposure.**

Scrubbing them from history properly would mean rewriting every commit from
`2343dad` onward and force-pushing over a public branch, on both `main` and
`gazebo-phase1`. That was **not** done, and it needs a decision rather than a
reflex, because:

- It is a force-push to a public repository, which is destructive and
  outward-facing, and it would happen in the hour before seven people clone
  that repository. A teammate who clones mid-rewrite gets a confusing state.
- **GitHub keeps unreferenced objects reachable by direct commit SHA** after a
  force-push, so the old commits can still be fetched by URL until GitHub
  garbage-collects them. Genuinely purging them means asking GitHub support.
  A force-push alone should not be reported as "removed".
- Anyone who has already cloned keeps their copy regardless.
- Every hash from `2343dad` onward changes, which invalidates the commit
  references in section 14 of this report and in the handoff itself.

The honest assessment: if the concern is casual discovery by someone browsing
the repository, this commit handles it. If the concern is that the content
exists on GitHub at all, only a rewrite plus a request to GitHub support
handles it, and it is better done after tonight's session than before it.

---

## 17. Blank-slate laptops, a published image, and three GPU tiers

**Written 2026-09-17, in the ninety minutes before the software session.** The
scope changed three ways from section 16: no USB drive, the image comes from a
registry; assume a **fresh Windows install** with nothing configured; and
**every machine must reach something runnable**, including laptops with no
discrete GPU and, as far as is honest, a MacBook.

Time was the binding constraint and it is worth recording what that cost. See
17.8 for what was not done.

### 17.1 The GHCR audit, and what Liam has to do by hand

**Publishing a public package is irreversible in practice, so the image was
audited first.** What was checked, and how:

| Checked | Method | Result |
| --- | --- | --- |
| Secret build arguments, private base images | `grep` the Dockerfile for `FROM`, `ARG`, `--mount=type=secret` | Clean. One `FROM ros:jazzy-ros-base`, no secret ARGs |
| The **layer history**, not just the Dockerfile | `docker history --no-trunc` over every layer | Clean. Only public apt installs from `packages.ros.org` and `archive.ubuntu.com`, and the ROS apt source deb is installed with a `sha256sum --strict --check` |
| Credential-shaped files anywhere on the filesystem | `find / -xdev` for `id_rsa*`, `*.pem`, `*.p12`, `.netrc`, `.git-credentials`, `.npmrc`, `authorized_keys` | Clean. The only `.pem` files are the CA root certificates in `/etc/ssl/certs`, which belong there |
| The usual secret directories | direct `ls` | `/root/.ssh`, `/root/.docker`, `/root/.aws`, `/root/.config/gh`, `/root/.gnupg`, `/root/.kube` all **absent**. `/etc/apt/auth.conf.d` exists and is **empty** |
| Shell history and build leftovers | `/root/.bash_history`, `/tmp/*`, `/var/tmp/*` | **None present** |
| ZED SDK or Stereolabs content | `find` for `*stereolabs*`, `libsl_zed*`, `/usr/local/zed` | **Absent.** This is the private-layer risk and it is not there |
| Token and private-key patterns | `grep -rIE` for `ghp_`, `github_pat_`, `AKIA`, `BEGIN PRIVATE KEY` over `/root /etc /opt /usr/local` | **No matches** |
| Authenticated apt repos | `grep` all of `/etc/apt/` for `://user:pass@` | **No matches**. Two sources, both public and both signed |
| Project source baked in | `find / -xdev -iname '*IGVC*'`, plus `ls` of the workspace | **Nothing.** `/root/ros2_ws/src`, `install` and `build` are all **absent** |

**The most useful single finding: the image contains no project code at all.**
The repository is bind-mounted at runtime, so the published artefact is ROS 2
Jazzy plus Gazebo Harmonic plus Nav2 and nothing of ours. That makes the
publication decision much easier than it would otherwise be.

The audit ran in a container started with **no volume mounts**, so the
bind-mounted repo could not pollute the result.

**Tagged, from the existing local image, no rebuild:**

```text
ghcr.io/wworth-igvc/igvc-gazebo-jazzy:2026-09-17
ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
```

Local image id `sha256:79115da25a4aee155701d9da6c2c7f8b56a878b38bbc4a1c9d4dad5ca6fe8a31`,
`os=linux arch=amd64`, 5.57 GB unpacked. **The registry digest does not exist
until the push happens**, so it is not recorded here; read it from the push
output or from `docker manifest inspect`.

**Two steps are Liam's and cannot be done from here.**

1. **Create a classic PAT.** `https://github.com/settings/tokens` >
   **Tokens (classic)** > **Generate new token (classic)** > tick
   **`write:packages`** and **`read:packages`** > **Generate token**.
   **It must be a classic token.** GitHub's own documentation states that
   Packages only supports authentication with a personal access token
   (classic); the fine-grained permissions reference has no Packages
   permission of any kind. This is a known trap and it wastes ten minutes if
   hit.
2. **Log in and push**, keeping the token out of shell history by pasting it at
   the prompt:

   ```powershell
   cd "C:\IGVC 2027\IGVC_robot_2027"
   docker login ghcr.io -u wworth-IGVC
   docker push ghcr.io/wworth-igvc/igvc-gazebo-jazzy:2026-09-17
   docker push ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
   ```

3. **Set the package visibility to Public.** Profile > **Packages** >
   `igvc-gazebo-jazzy` > **Package settings** > **Danger Zone** >
   **Change visibility** > **Public**. **A new package defaults to private,
   and a private package fails at pull time for every attendee with `denied`.**
   Note the package appears under the **user**, not the repository, until it is
   linked.

**DONE and verified 2026-09-17, after this section was first written.** The
push happened and the package is **public**, confirmed the way that actually
proves it: `docker logout ghcr.io` followed by an anonymous
`docker manifest inspect`, which returned the OCI image index with no
credentials and touched no local layers.

**Two digests, recorded separately because they differ and the difference
misleads:** the **index** digest is `sha256:79115da25a4aee155701d9da6c2c7f8b56a878b38bbc4a1c9d4dad5ca6fe8a31`, which is what Docker
reports as the repo digest and what `@sha256:` pins; the **amd64 manifest**
nested inside it is `sha256:d93b95a4021b898e52bf4f8c0ea1b2560ca3ea5c6a9efca1505246811ffdf0a3`, which is what `manifest inspect` prints.
Earlier text here gave only the first, unlabelled.

The index carries **amd64 only**, plus a second entry whose platform reads
`unknown/unknown`. **That is the buildx attestation, not a missing
architecture.** There is no arm64 entry, so macOS and Apple Silicon stay
unsupported exactly as documented.

**Three corrections to what this section originally said**, each of which cost
time on the day:

- **The GHCR account is `wworth-IGVC`**, the charlotte.edu account that owns
  the repository, not `CobaltTornado`. The package namespace is user-scoped, so
  **only the namespace owner can push to it.**
- **GHCR never accepts an account password, only a token.** Trying the account
  password presents as a credentials problem and is not one.
- **The repository-scoped package URL 404s.** The working public page is
  `https://github.com/users/wworth-IGVC/packages/container/package/igvc-gazebo-jazzy`,
  because the image was pushed from a local Docker rather than from Actions and
  so is not linked to the repository. **Whether a web page loads is not a test
  of public visibility**; the anonymous manifest fetch is.

**The pull time on a fresh machine is still NOT MEASURED**, because the package
existed only minutes before the session. The comparable figure that *is*
measured is 1.20 GB over the wire against 5.57 GB unpacked, from the earlier
`docker save` work.

**Why a registry rather than Docker Hub, and worth writing down:** Docker Hub
rate-limits anonymous pulls **per source IP**, so a room of people behind one
NAT shares a single allowance. GHCR does not limit anonymous pulls of a public
package the same way. That alone justifies the move independent of the USB
question.

**LAN fallback: documented, not built.** If GHCR is unreachable from the room,
the tarball can be served off Liam's laptop over HTTP. One section in the
session runbook, no scripts, with two honest cautions: WSL2's network address
is not the Windows address, so a `python3 -m http.server` inside WSL2 may need
a port proxy; and a campus network that isolates clients defeats it entirely,
in which case the real fallback is `BUILD_IMAGE=1`, which needs
`packages.ros.org` but not GHCR.

### 17.2 The tier matrix, with every measurement and every gap

`render_check.sh` had a defect that mattered for exactly this: it reported
hardware rendering only if the log matched `nvidia|geforce|rtx`, so **a machine
rendering in hardware on an Intel or AMD adapter fell through to
`RESULT: UNKNOWN`** and read as broken. It now extracts the adapter from
`GL_RENDERER`, names the tier, says what to do next, and writes the tier to
`/tmp/render_tier` so `bootstrap.sh` reads the verdict rather than re-deriving
it.

| Tier | Machine | Path | Status |
| --- | --- | --- | --- |
| **A** | Discrete NVIDIA | WSLg through D3D12 | **MEASURED.** The baseline |
| **B** | Integrated Intel or AMD only | WSLg through D3D12, plausibly hardware | **UNTESTED anywhere.** No such machine was available |
| **C** | Any Windows machine, software rendering | `LIBGL_ALWAYS_SOFTWARE=1` | **MEASURED today**, on this laptop with its GPU disabled |
| **D** | macOS | see 17.3 | **UNTESTED**, and partly refuted |

**Tier A, measured:**

```text
ADAPTER: D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)
RESULT: HARDWARE RENDERING (NVIDIA)
YOUR TIER: A          exit 0
```

**Tier B is honestly untested and the document says so rather than implying
equivalence.** What can be said without measuring: the mechanism is the same
D3D12 path, so a current WDDM driver should hardware-accelerate; and this
project's own notes record an Intel-adapter crash inside Intel's WSL driver on
a machine that has **both** an Intel iGPU and a discrete card, which is why
`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` is pinned in the compose file. A
pure-Intel machine does not hit that pin. `render_check.sh` now tells a tier B
user their GPU genuinely is rendering, and asks them to report numbers back.

**Tier C, measured today.** Method: 30 second windows, **counting messages**
rather than asking for a rate, with real time factor computed from `/clock`
rather than read from Gazebo's `real_time_factor`, which sits near 1.0 while
cameras crawl.

| Configuration | Real time factor | Camera delivered | Lidar delivered |
| --- | --- | --- | --- |
| 640x360 @ 15 Hz, front (the tier A default) | **0.608** | **1.63 Hz**, 11% of requested | 6.10 Hz |
| **320x180 @ 5 Hz, front (the recommendation)** | **0.854** | **4.27 Hz**, 85% of requested | 8.53 Hz |
| 320x180, `sim_cameras:=none` | **0.996** | none by definition | 9.95 Hz |

**And the result that decides whether tier C counts as runnable:
`bringup_smoke_test.sh` passed at 18 of 18, exit 0, under software rendering**,
with odometry 4.673 m against ground truth 4.681 m, heading disagreement
0.13 deg and distance ratio **0.9982**. So **a laptop with no usable GPU meets
tonight's target.** That is the single most useful number in this section.

Three things follow, and they are written into the quickstart's tier C path:

- **The camera is the entire cost.** Turning cameras off returns real time to
  0.996. Physics and lidar are fine on the CPU, which is why navigation,
  odometry and control work stay valid on tier C.
- **Ask for less and you get most of it.** At 320x180 and 5 Hz the simulator
  delivers 85% of requested frames; at the tier A default it delivers 11%.
- **`sim_cameras:=none` can never reach 18 of 18**, because the smoke test
  checks four camera topics. Tier C keeps the front camera, small.

The tier C recipe is **headless plus RViz, not the Gazebo window**, because the
Gazebo GUI is itself a rendered 3D view and competes with the simulation for
the same CPU.

**All of these are laptop-with-GPU-disabled numbers**, which is the same code
path a GPU-less machine takes but is not a measurement of any real tier C
machine, and none of them say anything about the AGX.

### 17.3 macOS, and two premises that did not survive

Researched rather than assumed, and each finding labelled.

| Claim | Verdict | What the evidence says |
| --- | --- | --- |
| Docker Desktop on macOS does not expose the GPU to Linux containers, so rendering is software only | **VERIFIED** | Docker's GPU documentation states that GPU support in Docker Desktop is only available on Windows with the WSL2 backend. The cause is architectural: containers run in a Linux VM on Apple's Virtualization framework, which provides no 3D-capable virtual GPU. There is no `--gpus` equivalent to try |
| Apple Silicon needs emulation for this amd64 image | **VERIFIED** | Docker's multi-platform docs: you cannot run a `linux/amd64` container on an arm64 host without emulation. Our image is single-arch amd64 |
| It needs `--platform linux/amd64` | **ASSUMED, overstated** | With only an amd64 manifest there is nothing for Docker to select, so `docker run` starts it under emulation with a platform-mismatch warning. Worth setting `platform: linux/amd64` in compose for predictability, but not a gate |
| It needs **Rosetta** | **REFUTED** | Docker's settings reference lists "Use Rosetta for x86_64/amd64 emulation on Apple Silicon" as **Disabled by default**. The default path is QEMU via `binfmt_misc`, which Docker's docs warn is much slower for compute-heavy tasks, and CPU rasterisation is exactly that. Rosetta also cannot advertise AVX/AVX2 to Linux guests, which matters because Mesa's LLVM JIT would fall back to narrower SSE paths |
| Emulation cost stacks on software rendering | **ASSUMED** | Mechanically sound, nobody has benchmarked gz-sim in this stack. Expect it to be bad; do not put a number on it |
| macOS has no WSLg equivalent | **ASSUMED** | An absence claim, so no single document asserts it. Docker Desktop for Mac ships no display-server integration, and Apple removed X11 years ago, which is why XQuartz exists separately |
| **XQuartz can display the Gazebo GUI** | **REFUTED, and this is the one that decides tonight** | XQuartz's GLX caps at roughly **OpenGL 1.4**; gz-sim's default `ogre2` engine requires **above 3.3, preferably 4.3+**. Enabling indirect GLX runs GL calls against that 1.4 stack, so `gz sim` **fails with an ogre2 OpenGL error rather than rendering slowly** |

**So the honest macOS position: tonight, a Mac owner pairs with a Windows
laptop on tier A or B.** Not because the checks would not run, but because the
one visual path everybody assumes works is a verified dead end. The headless
checks should run under emulation, since the autonomy path takes its lane lines
and barrels from `track_points.json` rather than from the camera, but that is
**ASSUMED**: nobody here has tested it on a Mac. If a visual path is ever
wanted, the route is an X server plus VNC or noVNC *inside* the container,
which rasterises client-side and never touches XQuartz's GL.

**Backlog note, scoped and not started: a native arm64 image is feasible, and
more so than expected.** Verified by downloading the real apt indexes rather
than reading documentation: REP-2000 lists Ubuntu Noble arm64 as **Tier 1 for
Jazzy**; `packages.ros.org/ros2/ubuntu/dists/noble/Release` declares
`Architectures: i386 amd64 arm64 armhf`; the arm64 index carries **3667
`ros-jazzy-*` packages** including `ros-jazzy-ros-gz`, `ros-jazzy-ros-gz-sim`
and `ros-jazzy-ros-gz-bridge`; and this image needs **no ZED SDK**, which is
the usual arm64 blocker. **Estimate: half a day**, mostly a `docker buildx`
multi-arch pipeline plus one round of verification on an actual Mac. **Not
started.**

### 17.4 The blank-slate prerequisite list, with sizes and times

Every `winget` package Id below was **verified by running `winget`**, not
recalled. Two of them are traps.

| Step | Id, verified | Download | Time | Reboot? |
| --- | --- | --- | --- | --- |
| Git for Windows | **`Git.Git`** 2.55.0.3 | about 65 MB | 1 to 2 min | no |
| WSL2 plus Ubuntu | **`wsl --install`**, not winget | about 500 MB | 3 to 5 min | **YES** |
| `wsl --update` | | about 130 MB | under 1 min | no |
| Docker Desktop | **`Docker.DockerDesktop`** 4.91.0 | about 600 MB | 5 to 8 min | **YES** |
| WSL Integration | a **UI step**, unscriptable | | 1 min | restarts Docker |
| `setup-windows.ps1` | must be run as `powershell -ExecutionPolicy Bypass -File ...`, because a fresh Windows defaults to **Restricted** | | under 1 min | no |
| Clone with 9 submodules | | about 600 MB | **17.4 s measured** | no |
| The image, pulled | | about 1.2 GB, 5.57 GB unpacked | not yet measured | no |

**Rough room total: about 2.4 GB per laptop, so five laptops is on the order of
12 GB.** On a shared uplink that is the binding constraint of the evening,
which is why the runbook starts downloads at minute zero and staggers the pulls
in pairs rather than running five at once.

**The two traps, both worth the ink:**

- **Do not install `Microsoft.Git`.** That Id is real, but it is Microsoft's
  fork build rather than the standard Git for Windows installer.
- **Do not install WSL from winget.** `Microsoft.WSL` exists (2.7.13), which
  contradicts the assumption that it does not, but `wsl --install` remains
  correct because only it enables the Windows optional features (Virtual
  Machine Platform, WSL) that WSL2 needs. An msix install does not.

Also verified: **`winget` itself is present by default on Windows 11** (this
machine reports v1.29.290) but on **Windows 10 it arrives via an App Installer
update** and is commonly missing or stale, so treat it as "verify first" there.
That matters because the third programmer is on Windows 10. Every step has a
manual download link for exactly this reason.

**Windows version thresholds, separated because they get conflated:**

| Build | Gates |
| --- | --- |
| 19041 | WSL2 itself |
| **19044** | **WSLg, that is, whether a Linux window can ever appear** |
| **19045** | what current Docker Desktop requires |

**Disk, re-derived for the pull path:** 5.57 GB image, 0.60 GB clone with
submodules, 0.003 GB colcon workspace, roughly 4 GB for Docker Desktop plus the
WSL2 Ubuntu distro. **About 10.2 GB.** The threshold is set at **20 GB**, and
**30 GB** to build, above the measurement rather than at it, because
`docker_data.vhdx` grows and never shrinks and Windows 11 Home has no Hyper-V
to compact it. `setup-windows.ps1` prints both the number it wants and the
number it found. The build figure is labelled a margin and not a measurement,
because 21.86 GB of this machine's 29.06 GB of build cache is shared across
four images and no single image's share is cleanly attributable.

**`setup-windows.ps1` never checked for Git at all**, which was odd for a
script whose purpose is getting a clone onto a blank machine. It now checks
Git and `core.autocrlf`, because `true` rewrites this repo's `.sh` files to
CRLF and every script then fails inside the container with `bad interpreter:
/usr/bin/env bash^M`.

### 17.5 Gate status on the merge result

| Gate | Result | Numbers |
| --- | --- | --- |
| `render_check.sh` | **PASS**, exit 0 | `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, `YOUR TIER: A`. Re-run after the tier rewrite |
| `bringup_smoke_test.sh` | **PASS**, exit 0 | **18 passed, 0 failed.** Robot moved 7.418 m on `/cmd_vel`, heading disagreement **0.13 deg**, distance ratio **0.9989**. Re-run on the final tip |
| `autonomy_check.sh` | **PASS**, exit 0 | From section 16: 83.0 m driven, max 1.86 m off the centreline, 0.0% over the paint |

**`autonomy_check.sh` was not re-run in this pass, and that is a deliberate,
checked decision rather than an omission.** `git diff --name-only
a7cf7b9..HEAD` restricted to the autonomy path returns nothing: no file under
`src/`, `config/`, `docker/`, the compose files, or `bringup_smoke_test.sh`,
`autonomy_check.sh`, `sim_preflight.sh`, `start_sim.sh`, `pose_logger.py` or
`generate_igvc_world.py` changed. The only script edits in this pass are
`render_check.sh`, `bootstrap.sh` and `setup-windows.ps1`, and neither of the
last two is in any gate's execution path. `render_check.sh` did change and was
therefore re-run.

**Additionally, and this is new evidence rather than a re-run:
`bringup_smoke_test.sh` also passes at 18 of 18 under forced software
rendering**, which is a strictly harder condition than the gate requires.

### 17.6 Documentation that was wrong before this pass

**C-21. Every USB and offline-transfer instruction is now wrong.** A sweep of
all 225 tracked files found **56 hits** across 8 files. The instructions are
removed from `GAZEBO_QUICKSTART.md` (an entire section 5 titled "Making the USB
drive", plus the recommended fast path, the timing table, the disk budget, the
sample output and the cheat sheet), `bootstrap.sh`, `setup-windows.ps1` (where
it was **runtime output**, not a comment), `README.md`, `GAZEBO_TODO.md`,
`CLAUDE.md` and the regenerated `.docx`. The **measurements** in section 16 of
this report are left as a historical record, because they were genuinely taken;
only the instructions are gone.

**C-22. `render_check.sh` reported a working Intel or AMD GPU as `UNKNOWN`.**
It matched only `nvidia|geforce|rtx`. Any machine rendering in hardware on an
integrated adapter got `RESULT: UNKNOWN` and an instruction to inspect the log
by hand, which reads as a broken machine. Given that tier B is a whole class of
teammate laptop, this would have produced a false failure in the room.

**C-23. `setup-windows.ps1` did not check for Git.** A prerequisite script for
getting a clone onto a blank machine, with no Git check and no `core.autocrlf`
check, while CRLF is a documented way to break every script in this repo.

**C-24. The quickstart began several steps after where a blank machine
actually is.** It assumed Git, Docker Desktop, WSL2 and a configured WSL
Integration toggle already existed, and opened at "get the repository". On a
fresh Windows install that is three installs and two reboots too late.

**C-25. The "XQuartz with `DISPLAY` set" advice for macOS was wrong**, and it
came from this task's own framing as well as general folklore. XQuartz's GLX
caps near OpenGL 1.4 and gz-sim's `ogre2` needs above 3.3, so it fails with an
OpenGL error rather than running slowly. Anyone sent down that path would have
spent the evening on a dead end.

**C-26. "Apple Silicon needs Rosetta" was wrong.** Rosetta is optional and off
by default in Docker Desktop; the default emulation path is QEMU
`binfmt_misc`. Related: `--platform linux/amd64` is good practice, not a hard
requirement.

**C-27. The disk thresholds were derived for the tar path and did not count
Docker Desktop or the WSL2 distro at all.** 12 GB became 20 GB for the pull
path, from a re-derivation that includes roughly 4 GB for the tooling a blank
machine does not have yet.

### 17.7 Clean-clone verification: NOT RUN, and no longer blocked

**Not run, and now for a different reason than when this was written.** Both
blockers have since cleared: `main` is pushed and the GHCR package is public
and anonymously pullable. What remains is simply that there was no time left
before the session to run it, and it must not be faked: cloning the local path
would verify something different from what was asked and would not exercise
the registry pull at all.

**So this is the first thing to do after the session**, not a permanent gap.
The sequence is below and it needs nothing from anyone else.

**What is verified, from section 16:** a clean clone with `--recurse-submodules`
into an isolated compose project, following only the written quickstart,
completed at **exit 0 in 201 s**, nine of nine steps, 18 of 18 topic checks,
with `.sh` line endings confirmed LF and shebangs intact. What that run does
**not** cover is the two things that changed today: the **GHCR pull** and the
**tier reporting**.

**The exact sequence to run after the push**, so it is not re-derived:

```bash
docker logout ghcr.io
docker manifest inspect ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest   # must succeed anonymously
cd /mnt/c/temp && rm -rf freshtest && mkdir freshtest && cd freshtest
time git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git clone
cd clone
COMPOSE_PROJECT_NAME=freshtest bash scripts/gazebo/bootstrap.sh
```

**Do not delete the working image to force a cold pull.** Section 16 records
that a remove-and-reload cost about **14 GB of `docker_data.vhdx`** that
Windows 11 Home cannot reclaim.

### 17.8 What was not done, and where this stopped

Stated plainly rather than left to be discovered.

- **`docs/DOCKER_CHANGES.md` has no GHCR section.** Section 15.3 still
  describes the offline path as the primary one. The pull-limit note and the
  VHDX growth warning are in the quickstart and in this section instead. This
  is the largest documentation gap from this pass.
- **`docs/GAZEBO_SETUP.md` was not updated** for the tier matrix or the
  `render_check.sh` rewrite. Sections 10.7 to 10.10 from the previous pass are
  still accurate; they simply do not mention tiers.
- **The clean-clone verification did not run.** 17.7.
- **The GHCR pull time is not measured**, because the package does not exist
  yet. 17.1.
- **`autonomy_check.sh` was not re-run.** Justified by diff in 17.5, not by
  assumption.
- **Tier B remains untested**, and no amount of writing fixes that: it needs an
  Intel or AMD laptop.

The `IN LANE` decision Liam made is carried: **`TOL` was not touched.** It is
described as a reported number and as provisional, with the reason stated in
both the quickstart and the runbook: the metric ignores yaw, and the pose log
it originally read was sampled at 1.92 Hz, which overstated the clearance
margin by at least 2.4x. No threshold gets tuned until the metric is right.

The `CLAUDE.md` decision is carried too: it stays **excluded and untracked**.
The distro line was corrected locally and the correction reaches nobody, which
is recorded as C-19 in section 16. Backlog: scrub the machine-specific paths
and the false "one ROS graph across two machines" claim before it is ever
tracked.

### 17.9 The push commands, in order

PowerShell, which is where these get run:

```powershell
cd "C:\IGVC 2027\IGVC_robot_2027"
git push origin main
```

**Do not hand a `/c/...` path to PowerShell.** It resolves it as a relative
path, tries `C:\c\IGVC 2027\...`, fails, and the `git push` on the next line
then runs in whatever directory you were already in and reports "not a git
repository". PowerShell 5.1 has no `&&`, which is why these stay two lines.

Then, and **in this order, because the second is what attendees hit**:

```powershell
docker login ghcr.io -u wworth-IGVC
docker push ghcr.io/wworth-igvc/igvc-gazebo-jazzy:2026-09-17
docker push ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
```

then **set the package visibility to Public in the GitHub UI**, then verify as
an outsider:

```powershell
docker logout ghcr.io
docker manifest inspect ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
```

That last command must succeed **with no credentials**. If it returns `denied`,
the visibility change did not take and every attendee's `docker pull` will fail
the same way.

---

## 18. A macOS path, built and verified as far as a Windows machine allows

**Written 2026-09-17 evening, as a background task while the software session
ran.** The goal was narrow: let a Mac owner run the simulator's automated
checks headless. A Gazebo or RViz window on a Mac is out of scope, for reasons
that are re-verified in 18.1.

Branch **`mac-support`**, cut from `main` at `f513273`. Nothing pushed.

### 18.1 The starting claims, each checked rather than assumed

| # | Claim | Verdict | Evidence |
| --- | --- | --- | --- |
| 1 | Docker Desktop on macOS gives containers no GPU, so all rendering is software | **VERIFIED**, externally | Docker's own GPU documentation states support is only available on Windows with the WSL2 backend. Not checkable from this repo; carried from the research in section 17.3 |
| 2 | The published image is **amd64 only** | **VERIFIED today** | `docker manifest inspect` returns an OCI index with exactly two entries: `amd64/linux` (`sha256:d93b95a4...`) and `unknown/unknown`, the buildx attestation. **No arm64 entry.** |
| 3 | XQuartz cannot display Gazebo; its GLX is below the renderer's OpenGL 3.3 | **CONCLUSION VERIFIED, NUMBER CORRECTED** | See below |
| 4 | `docker-compose.windows.yml` mounts WSL2 and WSLg paths that do not exist on macOS | **VERIFIED** | The `igvc_gazebo` service mounts `/usr/lib/wsl:ro`, `/tmp/.X11-unix` and `/mnt/wslg`, declares `devices: /dev/dxg`, and sets `gpus: all`. All four are Windows-only |
| 5 | `bootstrap.sh` uses that compose file | **VERIFIED** | It was hardcoded at line 41, `COMPOSE_FILE=docker-compose.windows.yml` |
| 6 | Tier C already passes: 18 of 18, odometry to 0.18 percent | **VERIFIED exactly** | The section 17.2 run reports `topic checks passed: 18 failed: 0` and `distance ratio 0.9982`, which is 0.18 percent |

**Correction to claim 3.** The brief says XQuartz's indirect GLX offers
"roughly OpenGL 2.1". The research in section 17.3 cited an open XQuartz issue
putting indirect GLX at **1.4**. Both figures circulate, they describe
different paths (direct versus indirect), and **both are below the OpenGL 3.3
that gz-sim's `ogre2` engine requires**, so the conclusion is unaffected and
the design is unchanged. The exact ceiling is **NOT VERIFIED here**: it needs a
Mac. Recorded because a number quoted confidently and wrongly is how this
project has lost time before.

**Two additions to the record, both found while checking.**

- **`shm_size` needed no work.** The brief asked me to check whether the
  Windows file already handles it. It does: `shm_size: "2gb"` is already there.
  The Mac file matches it rather than introducing it.
- **`bootstrap.sh` is already macOS-safe on its one GNU-only construct.** Its
  disk check runs `df -BG --output=avail`, which BSD `df` on macOS does not
  support, but stderr is suppressed and it falls through to
  `df -k . | awk '{print int($4/1048576)}'`. Field 4 is `Avail` on both GNU and
  BSD `df`, so the fallback is correct on macOS. Verified by reading; a
  grep for the other usual offenders (`readlink -f`, `sed -i`, `date -d`,
  `grep -P`, `stat -c`) returns nothing.

### 18.2 What was added

| File | What it does |
| --- | --- |
| `docker-compose.mac.yml` | **New.** The Gazebo service with every Windows assumption removed: no `gpus`, no `/dev/dxg`, no `/usr/lib/wsl`, no WSLg mounts, no `DISPLAY`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR` or `PULSE_SERVER`, no `GALLIUM_DRIVER`. Adds `platform: linux/amd64` and `LIBGL_ALWAYS_SOFTWARE=1`. Keeps `shm_size: "2gb"`. Container `igvc_gazebo_mac` and volumes `igvc_gazebo_mac_*`, distinct from the Windows ones |
| `docs/MAC_SETUP.md` | **New.** What works and what cannot, the install steps, the commands in order with expected output, where to record a failure, and the arm64 scoping note. Opens by stating that no Mac has run it |
| `docs/MAC_VERIFICATION_CHECKLIST.md` | **New.** A tear-off list for the Mac owner: command, what good looks like, and a blank for what actually happened, for twelve steps |
| `scripts/gazebo/bootstrap.sh` | **Extended, Windows behaviour unchanged.** `COMPOSE_FILE`, `SERVICE` and `CONTAINER` become `${VAR:-<the same default>}`, and a new `PULL_PLATFORM` defaults to empty so the Windows `docker pull` is invoked exactly as before. Verified: with nothing set, the three resolve to `docker-compose.windows.yml`, `igvc_gazebo`, `igvc_gazebo`, and `PULL_PLATFORM` is empty |

**The launch route, worked out from the scripts rather than assumed.**
`start_sim.sh` applies `HEADLESS=1` *after* `RVIZ`, so `HEADLESS=1` alone
forces `headless:=true rviz:=false` and overrides any `RVIZ` setting. More
usefully, **all three check scripts are already headless by default**:
`bringup_smoke_test.sh` and `autonomy_check.sh` both gate RViz behind
`${RVIZ:-0}`, and `render_check.sh` always runs `gz sim -s
--headless-rendering`, a server with no GUI at all. **So the Mac instruction is
simply "do not pass `RVIZ=1`", and no special headless flags are needed.**

`bootstrap.sh` **can** be reused rather than duplicated, which is the answer to
the question the brief left open. The macOS invocation is:

```bash
COMPOSE_FILE=docker-compose.mac.yml SERVICE=igvc_gazebo_mac \
CONTAINER=igvc_gazebo_mac PULL_PLATFORM=linux/amd64 \
bash scripts/gazebo/bootstrap.sh
```

It also tolerates tier C: `render_check.sh` exits 1 on software rendering by
design, and `RENDER_RC` is referenced only inside a warning message, so no
`fail` depends on it. Confirmed by grep.

### 18.3 Windows-side verification

`docker-compose.mac.yml` was run **on Windows**, project `macverify`,
`ROS_DOMAIN_ID=77`. With no GPU request and no display mounts it reproduces
the macOS condition in every respect **except processor architecture**, which
is exactly the part that cannot be tested here.

| Step | Command | Result | Wall clock |
| --- | --- | --- | --- |
| Parse | `docker compose -f docker-compose.mac.yml config` | **exit 0**, resolves; bind mount relative, `platform: linux/amd64`, `shm_size: 2147483648` | under 1 s |
| Start | `compose up -d igvc_gazebo_mac` | container **Up** | under 1 s |
| Leak check | `printenv` and `ls` inside | `DISPLAY`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR`, `GALLIUM_DRIVER` **all empty**; `/usr/lib/wsl`, `/tmp/.X11-unix`, `/dev/dxg` **all absent**; `ROS_DOMAIN_ID=77`, `LIBGL_ALWAYS_SOFTWARE=1` | |
| Build | colcon, 4 packages, **fresh volumes** | `build ok`, **exit 0** | **13 s** |
| Render | `render_check.sh` | `ADAPTER: llvmpipe (LLVM 20.1.2, 256 bits)`, `RESULT: SOFTWARE RENDERING`, **`YOUR TIER: C`**, exit 1 by design | **14 s** |
| Smoke | `bringup_smoke_test.sh` | **`topic checks passed: 18   failed: 0`**, exit 0. `DRIVE TEST : PASS` 4.377 m, `FRAME TEST : PASS`, heading **0.13 deg**, ratio **0.9981** | **116 s** |
| Autonomy | `autonomy_check.sh` | **`AUTONOMY CHECK: PASS`**, exit 0. 61.5 m driven under its own control, centreline deviation mean 0.68 m / max 1.78 m, worst clearance **+0.095 m**, sample rate 11.8 Hz. MOVED, PROGRESS, IN LANE and ON THE SLAB all PASS | **176 s** |
| No window | `ps -ef` for `rviz2` or `gz sim gui` | **no `rviz2` and no `gz sim gui` process at any point** | |
| Grep | resolved `compose config` for X11, WSLg, GPU | **no display mount, no device, no GPU reservation** | |

**Failures are loud, demonstrated on three deliberately broken inputs:**

| Broken input | Result |
| --- | --- |
| `COMPOSE_FILE=docker-compose.nope.yml bash scripts/gazebo/bootstrap.sh` | **exit 1**, stops at step 1, names the missing file and the cwd |
| `docker compose -f docker-compose.mac.yml up -d igvc_gazebo` (the Windows service name) | **exit 1**, `no such service: igvc_gazebo` |
| A scratchpad copy of the Mac file with a non-existent image tag | **exit 1** in seconds: `pull access denied ... repository does not exist`. **This validates the decision to omit `build:`**, see 18.5 |

### 18.4 Against the tier A baseline

| | Tier A, Windows compose | Mac compose on Windows |
| --- | --- | --- |
| Adapter | `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)` | `llvmpipe (LLVM 20.1.2, 256 bits)` |
| Tier | A | **C** |
| Smoke | 18 of 18 | **18 of 18** |
| Heading disagreement | 0.13 deg | **0.13 deg** |
| Distance ratio | 0.9989 | **0.9981** |

**Which differences are the absence of the GPU:** the adapter, the tier, and
camera throughput. Section 17.2 already measured the camera cost on this
machine: 1.63 Hz delivered against 15 Hz requested at the tier A default, and
0.608 real time.

**Which differences are not explained by the GPU at all:** none. **The
odometry numbers are effectively identical**, heading disagreement the same to
two decimals and the distance ratio differing by 0.08 percent, which is
run-to-run variation. That is the expected result and it is worth stating,
because it is the evidence that the physics and the odometry path do not care
about rendering. **A Mac should reproduce those two numbers.** If it does not,
that is a genuine finding and not an emulation artifact, which is why the
checklist asks for them specifically.

**What would need a Mac to explain:** any difference in the colcon build, any
timing, and anything involving `docker pull --platform`.

### 18.5 Assumptions made because you were unreachable

All reversible, all deliberate.

1. **No `build:` section in `docker-compose.mac.yml`**, unlike the Windows
   file. With one present, `docker compose up` on a machine that had not
   pulled the image would silently start building it, and on Apple Silicon
   that is an emulated multi-hour build that can fill a disk. Without it the
   same mistake fails in seconds, which fail test 3 demonstrates. **Reverse
   by pasting the `build:` block back.**
2. **`QT_QPA_PLATFORM=offscreen` is deliberately NOT set** in the compose
   file. Nothing in the default path starts a Qt application, and setting it
   globally would make an accidental `RVIZ=1` produce silence instead of a
   loud error. It is documented as a per-command escape hatch instead.
   **Reverse by adding one environment line.**
3. **`LD_LIBRARY_PATH` dropped entirely** rather than trimmed. On Windows it
   exists only to put `/usr/lib/wsl/lib` first; ROS's own `setup.bash` sets it
   correctly, and the image config sets none.
4. **`bootstrap.sh` extended rather than duplicated.** A separate
   `bootstrap-mac.sh` would have avoided touching the file at all, but it
   would drift. The `${VAR:-default}` form was judged to satisfy "extend only
   where an extension cannot change Windows behaviour", and that was verified
   by resolving the defaults with the variables unset.
5. **Names:** container `igvc_gazebo_mac`, volumes `igvc_gazebo_mac_*`. The
   distinct volumes matter more than they look: they are what let this be
   tested on Windows without touching the working Windows workspace.
6. **Test isolation:** project `macverify`, `ROS_DOMAIN_ID=77`.

### 18.6 What could not be tested, and the likeliest failure

1. **Whether the colcon build completes under emulation, and how long it
   takes. This is the most likely thing to fail** and it is step 6 of the
   checklist, deliberately before anything else interesting. Compiling C++
   under instruction translation is the heaviest thing in the sequence. It
   takes 13 seconds natively here and nobody knows the emulated figure.
2. **Rosetta versus the default QEMU path.** Rosetta is off by default and is
   not required. It also cannot advertise AVX or AVX2 to Linux guests, which
   is exactly what Mesa's software rasteriser benefits from, so it may be
   slower. The checklist asks which setting was used.
3. **macOS Docker Desktop itself**: its VM, its default resource limits, and
   whether `docker pull --platform linux/amd64` behaves as expected.
4. **Every timing.** Tier C on a Mac is a guess until measured.

### 18.7 The arm64 question, written only

Not started, per instruction. Full scoping is in `docs/MAC_SETUP.md` section 7.
In brief: a native arm64 image **removes the emulation layer and nothing
else**. Rendering stays on the CPU, because the GPU limitation is about Docker
on macOS rather than about architecture, **so there is still no window on a
Mac even with arm64.**

Feasible, verified from the real package indexes: REP-2000 lists Noble arm64 as
**Tier 1 for Jazzy**, `packages.ros.org` declares
`Architectures: i386 amd64 arm64 armhf`, the arm64 index carries **3667
`ros-jazzy-*` packages** including `ros-gz`, `ros-gz-sim` and `ros-gz-bridge`,
and **this image needs no ZED SDK**, which is the usual blocker here. It needs
a `docker buildx` multi-architecture build and somewhere to push both
manifests, which GHCR already supports under the existing tag.

**Estimate: half a day** with native arm64 build hardware, materially longer
cross-built under emulation. **Do not start it before a filled-in checklist
comes back**: if emulation is tolerable this is a nice-to-have, and if the
emulated build fails outright it becomes the only way a Mac participates.

### 18.8 Commits and the push command

Branch `mac-support`, cut from `main` at `f513273`. **Nothing pushed.**

| Commit | What |
| --- | --- |
| `94c9506` | Add a macOS compose file, headless only, with the display plumbing removed |
| `952b92f` | Let bootstrap.sh take a compose file, service and container, Windows unchanged |
| `4c22497` | Document the macOS path, and ask the Mac owner for the data nobody has |

plus the commit carrying this report section.

```powershell
cd "C:\IGVC 2027\IGVC_robot_2027"
git push -u origin mac-support
```

`main` is untouched by this work. Nothing here changes any Windows behaviour,
and that claim is backed by the default-resolution check in 18.2 rather than
by inspection alone.
