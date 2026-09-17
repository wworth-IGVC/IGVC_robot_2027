# Gazebo against the 2026 robot as designed — AGX Orin baseline

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
   (`navigator.py:1702`). That counter is `min(x + 1, 4)` — it **saturates at 4**
   (`navigator.py:1595`, `:1644`, `:1684`) and **resets to 0 on success**
   (`:1675`). It also counts goal *rejections*, and explicitly does **not**
   count `STATUS_CANCELED` ("expected when we replan"). So `aborts=4` means "at
   least four consecutive failures at the instant it was sampled". The true
   total is unknown. My run today printed `aborts=1`. Nothing in
   `autonomy_check.sh` counts aborts at all — it echoes that string once
   (`:99-100`).

2. **The robot's camera and its planner are looking at two different courses.**
   `generate_igvc_world.py:137-138` paints a **constant 12 ft (3.658 m)** lane
   into `igvc_course.sdf`. Nav2's corridor does not come from that file at all —
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
   `lane_detection.py` imports **no torch** (grep count 0); it is classical CV —
   CLAHE, HSV gate, Canny, `HoughLinesP`. The node that hard-fails without torch
   is **`lane_segmentation_node`** (YOLOPv2). Related: `STATUS.md:25` says
   `num_cameras: 3` makes "the synchroniser never fire and log nothing".
   `lane_detection.py:81-91` builds **one independent synchroniser per camera**,
   so camera 0 fires normally on the front camera; only 1 and 2 starve. And it
   **does** log — a 2 s watchdog with a throttled WARN (`:119`, `:924-929`).

---

## 1. Regression baseline, measured today

All three gates pass. These are the numbers any later phase must not regress.

| Check | Result | Evidence |
| --- | --- | --- |
| `render_check.sh` | **PASS**, exit 0 | `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`, Mesa 25.2.8; camera and gpu_lidar both `PUBLISHING` |
| `bringup_smoke_test.sh` | **18 passed, 0 failed**, exit 0 | odom 7.796 m vs gz truth 7.804 m; heading disagreement **0.13°**; distance ratio **0.9989** |
| `autonomy_check.sh` | **PASS**, exit 0 | 51.3 m driven, span 23.1 m, centreline deviation mean 0.54 m / max 1.54 m, on the slab throughout, `aborts=1` |

Environment: Docker 29.7.2, 24 CPUs, 16.4 GB to the VM. **123.6 GB free on C:**
— well above the 40 GB floor. `docker system df`: 68.9 GB images (33.6 GB
reclaimable), 29.1 GB build cache.

### Two de-risking probes I ran because they change the plan

**torch on Blackwell — PASSES.** `igvc-humble-fused-drive`: torch
**2.14.0+cu130**, CUDA 13.0, cuDNN 9.24, `is_available: True`, device
`NVIDIA GeForce RTX 5070 Ti Laptop GPU`, capability `(12, 0)`, and
`arch_list` contains **`sm_120`**. A real convolution ran on the GPU:
`CONV_FORWARD_OK (1, 16, 224, 224) sum=24495.5254`. ultralytics 8.4.146,
cv2 4.5.4, numpy 1.26.4. **`onnxruntime` is absent.**

**Cross-container plumbing — better than expected.** Both containers sit on
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
  documented command — `bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh`
  — fails on a clean container with `gz: No such file or directory` and reports
  `RESULT: UNKNOWN`. One-line sim-side fix.
- **`sim_cameras` validates nothing.** `gazebo_sim.urdf.xacro:209,212` are two
  `xacro:if`s with no else. Any typo (`fron`, `front,left`, `true`) silently
  produces a robot with **zero cameras** that launches cleanly.

---

## 2. Deployment map, and the topology contradiction — resolved

The contradiction was: the design report says the Jetson is the brain and sends
CAN; `SIM_WORK_RESUMPTION.md` infers a laptop because `igvc_jetson_stack` is
commented out.

**Both were reasoning from a file that did not launch the competition run.**
`upstream/more_diverging_changes:scripts/auton_launch.sh:3-4` runs
`docker start dazzling_easley` then `docker exec -it dazzling_easley`.
`dazzling_easley` is a Docker-generated random name — created by a bare
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
host` plus `privileged: true` — which both active services have — is exactly
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
| Obstacle detection | **See §3 — the report and the code disagree.** | Not running; obstacles come from `track_points.json` | Nav2 stock `ObstacleLayer` on bridged `/scan` + clouds |
| Odometry | ZED depth + IMU + LiDAR fused with GPS "using a ZED ROS 2 wrapper" (report p.15) — technically implausible, see §3 | `gazebo_odom_shim` rotating Gazebo odometry by the spawn yaw | Unchanged |
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
| …loaded through the ZED ROS 2 wrapper | **Nothing in the repository consumes the ZED wrapper's detection output** — zero subscribers to `obj_det`/`ObjectsStamped`/`zed_msgs` across all 16 branches. `od_enabled: false` in both configs the rest of the repo uses. The one enabled config, `zed2i_custom_detection.yaml`, is referenced by a launch file that exists **only on branch `2026/zed`**, and its `custom_onnx_file` points at `/home/root/ros2_ws/.../best.onnx` — a path that does not exist and is malformed (`/home/root`). |
| LiDAR data "fused with GPS using a ZED ROS 2 wrapper" (p.15) | The ZED wrapper has no LiDAR ingest. The LiDAR is a USB/J40 peripheral in the schematic. **The report's most implausible claim.** |
| "negligible difference" between sim and reality (§8.5) | Three lines of text, zero data, no sim-vs-real measurement. It was a claim about Isaac streaming HD1200 into the real ZED SDK. It does not transfer to Gazebo's 640×360 pinhole RGB. Quoting it for Gazebo would be a category error. |
| RPLIDAR **C4** (p.12) | **C1** throughout: `rplidar_c1.urdf.xacro`, `top_rplidar_c1_link`, sensor `rplidar_c1` |
| Self-Drive functions: **sixteen** (§7.2) and **twenty-one** (§1.2.3, §7.2) | Both appear, four lines apart, plus "sixteen completed in one run" which equals the total asserted elsewhere. |
| Nav2 with "SmacHybrid Planner and Regulated Pure Pursuit" (p.15) | Planner is `SmacPlanner2D` (2D, not Hybrid); controller is RegulatedPurePursuit. But the report's *mechanism* description — sampling carrot distances over a distribution and scoring candidates on speed/obstacle-distance/progress — describes DWB or MPPI, not RPP. |
| No wheel radius, no resolution, no frame rate anywhere | Grep across all 19 pages: `radius` 0 hits, `fps`/`Hz` 0 hits, `resolution` 0 hits. The report gives RQ-02 no help at all. |
| Cover: "Date Submitted: May 15, 2025"; body: "the 2026 competition rules" | — |
| §5.2.2: "Images will be attached below" showing detection results | `p.15` and `p.16` contain **zero embedded images**. |

**What this means for the added obstacle-detection item.** The real
detection-to-costmap node is `obstacle_costmap.py`, which consumes
`/yolo/detections_3d` from **`yolo_ros`, not the ZED SDK** — and it exists only
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
| **URDF collision geometry** — chassis mesh bbox under `rpy yaw +90°` | **0.810 m** (`test_robot_body.urdf.xacro:203-208`) |
| Nav2 footprint (both costmaps) | **0.700 m** (`nav2_lane_follow_config.yaml:250`, `:354`) |
| collision_monitor StopZone | 0.800 m (`:176`) |
| collision_monitor SlowZone | 1.000 m (`:169`) |
| Design report | 0.762 m (2.5 ft) |

The **chassis** is the widest part, not the wheels — the wheels are ~0.17 m
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
**negative** — *there is no zero-cost cell anywhere in that gap*. A 0.810 m
robot does not fit in 0.667 m; a 0.700 m one barely does.

Widening the footprint to the true 0.81 m raises the inscribed radius to 0.405
and **makes this worse before it makes it correct**. That is still the right
trade, but it is a deliberate decision, not a drive-by fix.

**Three config comments that contradict their own values** (all in
`nav2_lane_follow_config.yaml`, none of which I will edit — they are real-robot
config):

- `:315-316` "lethal_cost_threshold is set **ABOVE 100** so lane cells come in
  as high-cost (253) rather than lethal (254)" — the value on `:325` is **90**,
  and `gt_nav_bridge_node.py:274` writes lane cells as exactly **100**. So
  100 ≥ 90 and they become **LETHAL (254)** — precisely the failure the comment
  says forced `use_collision_detection: false`. The same comment reasons about
  MPPI; the configured controller is RegulatedPurePursuit.
- `:331-339` titled "Reduced from 0.75m", reasons to "0.50m", and the value two
  lines later is **0.75**.
- `:363` sizes the global obstacle ranges "to match the 30×30 m window"; the
  global costmap is **100 × 100 m**.

Also worth knowing: **collision_monitor's point-cloud sources are all
`enabled: false` in the shipped real config** (`:189`, `:197`, `:205`), so its
stop and slow polygons can never trip — on the real robot, not just in sim.

---

## 5. Status table

`VERIFIED` = a command ran and a number came back. Nothing here is `VERIFIED`
yet except the baseline, because Phase 0 stops before the work.

| # | Item | Status | One-line reason |
| --- | --- | --- | --- |
| — | Regression baseline (render / smoke / autonomy) | **VERIFIED** | 18/18, autonomy PASS, GPU confirmed — §1 |
| — | torch on Blackwell | **VERIFIED** | `sm_120` present, real conv ran on GPU — §1 |
| P0-1 | Get perception running | **PLANNED** | Path is clear; the to-do list names the wrong node (§0.4) |
| P0-2 | `num_cameras` sim override | **VERIFIED, not needed** | `/lane_map` carries 253 occupied cells with camera 0 alone; one startup WARN only (§10.7) |
| P0-3 | Camera point cloud orientation | **PLANNED** | No prior research exists on this; pass 2 never discusses optical frames |
| P1-4 | RQ-06 control path | **TEAM DECISION** | Brief to write; pass 2 recommends `gz_ros2_control` |
| P1-5 | FollowPath aborts | **PLANNED, premise corrected** | The "four" is a saturating gauge (§0.1); corridor is a *hypothesis*, not a diagnosis (§9 C-3) |
| P1-6 | `render_check.sh` on other machines | **OUT OF REACH** | Runbook only — no access to the 5080 or the Win10 machine |
| P1-7 | Verify `igvc_gazebo_linux` | **OUT OF REACH** | Runbook only — no Linux host |
| P1-8 | RQ-11 barrel physics | **TEAM DECISION** | Brief to write |
| P1-9 | Publish `/fix` | **PLANNED** | Blocked on a datum decision and a topic-name decision (§6) |
| P2-10 | `/cmd_vel` timeout | **PLANNED** | Smoke test already reports "DiffDrive held the last command" across 4 s |
| P2-11 | Four zero-normal meshes | **PLANNED** | Staging-directory repair + hash comparison |
| P2-12 | `sim_cameras:=all` on the real course | **PLANNED** | Blocked by a launch-file gap (§6) |
| P2-13 | YOLOPv2 at 640×360 | **PLANNED, premise corrected** | It will never crash — the frame is upscaled to 1280×720 first |
| P2-14 | `wheel_radius` | **DOCUMENT ONLY** | Sim corroborates 0.1016; ratio 0.9989 today |
| ADD-15 | Obstacle detection inventory | **VERIFIED (inventory)** | §3 — 15 classes not 13, nothing consumes ZED detections, `best.pt` already on disk |

---

## 6. The plan

Order, the test for each, the number I expect, and what would make it fail.

### Phase 1 — P0

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
has the dict *before* the yaml, so the yaml wins and nothing is overridable —
which is why a new file is cleaner than editing that one.

**1.3 Point cloud orientation.** Barrel at a known world pose; transform the
cloud into `base_link` through its own `header.frame_id`; assert the fitted
floor normal is within 5° of +Z and the barrel clusters at the expected range.
*Prove it can fail* by running once with a deliberately wrong frame.
*Expect:* either clean pass, or a 90° error — `gz_frame_id` is the optical
frame, which is right for the rasters and may be wrong for the cloud.

**1.4 Lane output — and this one needs a decision.** As specified, it cannot
work: §0.2 (two different courses) and §0.3 (complementary encodings). My
proposal, all sim-side: a new `scripts/gazebo/lane_eval_sim.py` that renders
lane-line ground truth from **the same `track_points.json` geometry
`generate_igvc_world.py` paints into the world**, and scores `/lane_map`
against that. It reuses the exact geometry the camera sees, which is the whole
point. `lane_eval_node.py` is in `src/igvc_lane_detection/` and stays untouched.

### Phase 2 — P1

**2.1 FollowPath aborts.** First *instrument* it: count `STATUS_ABORTED` results
from the log rather than sampling a saturating gauge, and log pose + nearest
obstacle + costmap cost along the path at each. Then ≥3 runs per condition
(single runs have misled this project twice). The corridor arithmetic in §4 is
already the hypothesis. All hypothesis tests go through a clearly-named sim-only
overlay; `nav2_lane_follow_config.yaml` is not edited.

**2.2 `/fix`.** `<spherical_coordinates>` into the **generator**, plus a
`navsat` sensor on `front_gps_antenna_link` (which is at `0.177 -0.133 0.218`
relative to `base_link` — note the antenna is 13.3 cm right of centreline, a
real lever arm), plus a bridge row. *Test:* drive a known displacement, compare
NavSatFix-derived local ENU against Gazebo ground truth including heading, so a
rotated datum fails. **Trap:** `igvc_course.sdf` is *committed* and only
regenerated when `track_points.json` is newer (`start_sim.sh:70`), so the
regenerated world must be committed in the same change or nobody's checkout
changes.

**2.3 and 2.4** — RQ-06 and RQ-11 briefs under `docs/decisions/`.
**2.5** — runbooks under `docs/runbooks/` for the 5080 laptop, the Windows 10
machine, and native Linux.

### Phase 3 — P2

`/cmd_vel` timeout with ≥5 repeats reporting a distribution, judged on Gazebo
**ground-truth pose** with an asserted on-slab precondition. Mesh repair into a
staging directory with hashed vertex/topology comparison. `sim_cameras:=all` at
several resolutions. `wheel_radius` documented only.

### Three decisions I need from you before Phase 1

1. **The lane-evaluation approach** (§0.3 / 1.4). My recommendation is the new
   sim-side evaluator. The alternative — making `/lane_ground_truth` emit 100
   for lane cells — is a forbidden edit *and* riskier, because
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
  4.6° too narrow or **26° too wide** — the sign is unknown. Every sim-vs-real
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
  So the autonomy path is locked to 640×360 at 15 Hz — which blocks exactly the
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

1. **On the robot's Jetson, run `docker ps -a` — is there a container called
   `dazzling_easley`?** That name is hardcoded in all five of Nitin's
   competition scripts. Its existence settles the topology in thirty seconds
   and does not depend on anyone remembering June correctly.
2. You said "the Dockerfile on `main`" ran at competition. Could the **image**
   have been built from `main` while the **code** bind-mounted into it was
   `more_diverging_changes`? That would reconcile your answer with the commit
   history rather than contradict it.
3. **Which lens is fitted to the three ZED X units — 2.2 mm or 4 mm?** (Or: does
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

1. "Four FollowPath aborts in 143 s" — **withdrawn**, it is a saturating gauge.
2. "The robot is 0.70 m wide" — **withdrawn**, that is the Nav2 footprint; the
   collision model is 0.810 m.
3. "`lane_detection_node` cannot start without torch" — **withdrawn**, wrong
   node; it imports no torch.
4. "The synchroniser never fires and logs nothing" — **withdrawn**, one
   synchroniser per camera, and there is a watchdog that logs.
5. "One ROS graph across two machines" (`CLAUDE.md`, `SIM_WORK_RESUMPTION.md`) —
   **withdrawn**, the two compose files set different RMW implementations.
6. "The design report describes a 13-class model" — the model has **15**.

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
| **`wheel_radius`** | **0.2032** (flagged wrong) | **0.1016** | **RQ-02 is effectively answered on MDC** |
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
