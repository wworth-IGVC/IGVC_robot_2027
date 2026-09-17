# Session handoff

**Written 2026-09-17.** Read this first. It assumes you know nothing about this
work. Everything here was measured, not inferred, unless it says otherwise.

---

## 1. The brief

Make the 2026 robot and its simulator run in **Gazebo** the way the 2026 robot
was designed to run, before changing anything, treating the robot as still
carrying its **Jetson AGX Orin**. This is a parity and portability job, not a
feature job: the 2026 stack is not modified, only the simulation around it.

**Today's immediate goal outranks that**: `main` has to be ready for the
software sub-team to clone at a **6 to 8 pm meeting tonight**. That work has not
started and arrives as a separate prompt.

---

## 2. Repository state

Working directory, which **contains a space, so quote every path**:

```text
C:\IGVC 2027\IGVC_robot_2027          the repo
C:\IGVC 2027                          session root, research docs, NOT a git repo
```

Branch **`gazebo-phase1`**, created from `main` (`8a38fbc`). `git status -sb`
at handoff: clean apart from what the final commits below cover.

Commits on `gazebo-phase1`, oldest first:

| Hash | Message |
| --- | --- |
| `d005781` | Make render_check.sh work on a clean container |
| `8ee7e0e` | Reject an unrecognised sim_cameras instead of building a robot with none |
| `8e773b2` | Let the autonomy path set the camera resolution |
| `97a9704` | Paint the course the planner actually plans in, not a constant 12 ft lane |
| `961b99f` | Report how close the robot came to the painted line, not just to the centreline |
| `0338dbc` | Record the AGX-baseline audit: Phase 0, the corrections, and Phase 1 |
| `4a82b34` | Read the lane width from the track generator instead of copying it |
| `6902e2f` | Measure lane clearance from the rotated footprint, and sample fast enough to see it |
| `605a795` | Bring the to-do list and the Docker record up to date |
| `7eb3339` | Prove the camera point cloud is rotated 90 degrees, and ship the check |
| `e67a106` | Add the sim-side lane evaluator, and the first non-zero lane numbers |
| `79e4415` | Keep the scripts that produced the numbers in the report |
| `2343dad` | Record the three P0 results, and write a handoff a fresh session can boot from |

`git log --oneline main..HEAD` is authoritative if this table has drifted.

- **`main` is unchanged.** Nothing has been merged into it.
- **`docs-trim` is untouched.** It carries five unpushed commits of unknown
  provenance that restructure `docs/` (they create `docs/archive/`,
  `docs/STATUS.md`, `docs/GOTCHAS.md`, `docs/REFERENCE.md`). Nobody on this side
  wrote them. Do not rebase onto it and do not merge it without asking Liam.
- **Nothing has been pushed.** Liam runs every push.

Push, in order:

```bash
cd "/c/IGVC 2027/IGVC_robot_2027"
git push -u origin gazebo-phase1
```

---

## 3. Environment

Docker Engine **29.7.2**, Docker Desktop on Windows 11 with the WSL2 backend,
WSL2 distro **Ubuntu-26.04**. **112.5 GB free on C:** at handoff; stop and ask
below 40 GB. Docker Desktop stops when the machine sleeps; restart it from
`C:\Program Files\Docker\Docker\Docker Desktop.exe` and wait for `docker info`.

| Container | Image | State at handoff | What it is |
| --- | --- | --- | --- |
| `igvc_gazebo` | `igvc-gazebo-jazzy:latest`, 5.57 GB | up, idle | ROS 2 **Jazzy** + Gazebo Harmonic, gz-sim 8.15.0. The simulator. |
| `igvc_humble_fused_drive` | `igvc-humble-fused-drive:latest`, 14.1 GB | up, idle | ROS 2 **Humble** + torch. The **Jetson stand-in** for perception. |

Other images present: `igvc-zed-humble:latest` and `:upstream` (28.6 GB each),
`igvc-gazebo-harmonic:latest` (4.24 GB, deprecated), `ros:jazzy-ros-base`.

**Start the simulator** (from a **WSL2** shell, never PowerShell, or there is no
GUI; headless works from anywhere):

```bash
cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
docker compose -f docker-compose.windows.yml up -d igvc_gazebo
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

The repo is bind-mounted inside the container at
`/root/ros2_ws/src/IGVC_robot_2026`; that path keeps saying **2026** on purpose.

**Sourcing matters.** `gz` comes from the ROS *vendor* packages
(`/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`) and `/root/.bashrc` does **not**
source ROS, so a bare `docker exec` has no `gz` and no `ros2`:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash    # after a colcon build
```

---

## 4. The three gates: the regression baseline

Run all three inside `igvc_gazebo`. A regression in any of them blocks work.

| Gate | Most recent result | What it catches |
| --- | --- | --- |
| `scripts/gazebo/render_check.sh` | **exit 0**, `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)` | Silent fall back to CPU rendering, which invalidates every camera result |
| `scripts/gazebo/bringup_smoke_test.sh` | **18 passed, 0 failed**, odom 8.588 m vs truth 8.597 m, heading 0.13 deg, ratio 0.9990 | Every contract topic carries a message; camera frame ids resolve in TF; odometry agrees with ground truth in heading and distance |
| `scripts/gazebo/autonomy_check.sh` | **PASS**, 47 to 51 m driven, 0.0% of samples over the paint | The robot drives the course under its own control, never touching `/cmd_vel` |

**Two caveats on the baseline, both real:**

1. **`autonomy_check.sh`'s `IN LANE` gate is a coin flip.** At the honest 15 Hz
   sampling it produced **2.04 (FAIL), 1.95 (PASS), 1.98 (PASS)** against a flat
   2.00 m tolerance. The behaviour is fine; the gate measures the wrong quantity
   (a constant tolerance against a lane whose width varies by a factor of two)
   and sits on its own threshold. **Liam has been asked to decide; do not raise
   `TOL` to make it green.**
2. **`bringup_smoke_test.sh` PART 2 can drive off the slab.** The spawn is only
   **9.44 m** from the slab edge along the drive heading, and DiffDrive holds the
   last command for 4 to 5 m after `/cmd_vel` stops, so total travel runs 7.8 to
   11.3 m. One run ended at world z = **-118.9 m**, falling. The test's own
   on-ground guard caught it and refused a verdict, which is the guard working.
   **This is pre-existing**: the slab geometry is byte-identical to `main`.

---

## 5. The rules. Restated in full, because you do not have them

**Forbidden to edit.** These are the 2026 stack and the real robot:

- `src/igvc_lane_detection/` and anything it imports
- `nav2_lane_follow_config.yaml` and its siblings
- `controllers.yaml`
- `lane_detection_config.yaml`, `lane_segmentation_config.yaml`
- the real robot URDF and xacro under `urdf/parts/` and `urdf/robots/`, and the
  meshes
- anything on the `CanInterface` / ODrive path

**Sim-side and allowed:**

- `scripts/gazebo/` including `measurements/`
- `scripts/gazebo/generate_igvc_world.py` and the world it generates
- `src/igvc_test_description/urdf/gazebo/gazebo_sim.urdf.xacro`
- sim launch files (`gazebo_sim.launch.py`, `gazebo_nav_test.launch.py`)
- `config/gazebo_bridge.yaml`, `gazebo_odom_shim`
- documentation

**Process:**

- **The freeze rule: nothing merges into `main` unless all three gates pass on
  the merge result.**
- **Liam runs every `git push`.** Prepare commits, hand over the command.
- **Never ask for a credential**, and never accept one pasted into chat.
- **Do not commit unverified work.** "Done" means a command ran and a number
  came back; record both.
- **Every new check must be shown to fail** on a deliberately broken input.
- **Laptop numbers are never presented as AGX performance.** Everything measured
  here is x86 on an RTX 5070 Ti Laptop. Label it.
- **No em dashes** in any document.
- One topic per commit, long explanatory messages, `git commit -F` from a file
  because heredocs mangle apostrophes. End every message with
  `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.

---

## 6. Claims that are known false

You will otherwise re-trust these and lose a session.

| Claim, and where it is written | The truth |
| --- | --- |
| `CLAUDE.md` and `SIM_WORK_RESUMPTION.md`: the 2026 setup was "one ROS graph across two machines" | **False.** `docker-compose.yml` sets `rmw_fastrtps_cpp`, `docker-compose.jetson.yml` sets `rmw_zenoh_cpp`, and the Zenoh config listens only on `tcp/localhost:7447` with the cross-host endpoint commented out. They could not have formed one graph. |
| `GAZEBO_TODO.md` lists `render_check.sh` under "Done, and verified" | It **failed on a clean container** until `d005781`. It never sourced ROS and `gz` is not on PATH until you do. |
| "Four FollowPath aborts in 143 s" | A **saturating gauge**, `min(x+1, 4)`, reset to 0 on success, also incremented on goal rejection. Not an event count. Nothing counts aborts. |
| "The robot is 0.70 m wide" | That is the **Nav2 footprint parameter**. The chassis collision mesh is **0.810 x 0.970 m**, and the chassis is wider than the wheels. |
| The torch item names `lane_detection_node` | `lane_detection.py` imports **no torch**; it is CLAHE, Canny and `HoughLinesP`, and it runs in the Gazebo image today. **`lane_segmentation_node`** is the one that needs torch. |
| `lane_eval_node` measures lane quality | It has scored **identically zero since 2026-05-14 11:12**. It thresholds at 50; `track_ground_truth_node` writes only 0 and -1. And the two grids are semantically complementary, a filled corridor against painted lines. |
| The GPS waypoint `42.400510946, -83.130518968` is the competition venue | **31.1 km away** from Oakland University. Do not use it as a datum labelled "venue". |
| `main` is the code that competed | Probably not. `more_diverging_changes` received 54 of the 55 competition-window commits and carries the venue GPS launch files. **Adopting it is a port, not a checkout**: it deletes the entire simulator, restores the four zero-normal mesh colliders that segfault dartsim, and rewrites every mesh URI to `/home/nitin-5090/...`. |

---

## 7. Where everything lives

In the repo:

| Path | What |
| --- | --- |
| `docs/GAZEBO_AGX_BASELINE_REPORT.md` | **This work's report.** Phase 0, both correction rounds, Phase 1, the P0 results, the branch diff, the questions for Aidan |
| `docs/GAZEBO_TODO.md` | The to-do list, updated, with the retractions |
| `docs/GAZEBO_SETUP.md` | System of record for the simulator |
| `docs/DOCKER_CHANGES.md` | Image and compose history; section 14 is this session's measured image facts |
| `docs/RQ03_AUDIT.md` | What downstream nodes subscribe to, and four silent failures |
| `docs/GAZEBO_QUICKSTART.md` | Zero to a driving robot |
| `docs/IGVC_2027_Gazebo_Setup_Guide.docx` | Generated from the quickstart by `docs/make_gazebo_docx.py`. Edit the Markdown |
| `scripts/setup-windows.ps1` | Windows machine setup |
| `scripts/gazebo/` | The gates, `start_sim.sh`, `sim_preflight.sh`, the world generator |
| `scripts/gazebo/generate_igvc_world.py` | Builds `src/igvc_test_description/worlds/igvc_course.sdf` from `track_points.json` |
| `scripts/gazebo/measurements/` | This session's measurement scripts, see section 9 |

In the session root `C:\IGVC 2027`, **not** in git. Their real filenames use
spaces where the docs cite underscores, so match on the pattern:

| Pattern that matches on disk | What |
| --- | --- |
| `CLAUDE.md` | Project orientation. Its routing table is stale for `DOCKER_CHANGES.md` and `RQ03_AUDIT.md` |
| `SIM_WORK_RESUMPTION.md` | Routing and state. Has no sections "5.1" or "5.5" despite being cited that way |
| `RESEARCH_QUESTIONS_AND_UNRESOLVED.md` | 25 numbered RQs. Sections 1 to 4 are verified from code. Predates the Jazzy move |
| `GRR_REPO_MAP.md` | The org and its 48 repos |
| `TASK_RESUMPTION.md` | Docker history. Says "consolidate on Humble", which is superseded |
| `gazebo bi directional research pass 2.md` | Research answers. Predates Jazzy; its `gz_ros2_control` source build is obsolete |
| `UNCC charlotte last year IGVC design report.pdf` | The 2026 design report |
| `IGVC - 2026rules.pdf` | Rules authority. II.2 is the AutoNav course spec |

---

## 8. Environment traps that have already cost time

- **`MSYS_NO_PATHCONV=1`** before any `docker cp` or `docker run -v`, or Git Bash
  rewrites `/tmp/x` into `C:/Program Files/Git/tmp/x`. But **unset it for git**,
  which then cannot read a `/c/...` path.
- **Quote the repo path**, always. It contains a space.
- **Write scripts with the Write tool**, not heredocs: the Bash tool wraps
  commands in `eval` and mangles apostrophes and backslashes.
- **`set -o pipefail`, never `set -u`.** `set -u` breaks
  `/opt/ros/$ROS_DISTRO/setup.bash`, which reads `AMENT_TRACE_SETUP_FILES`
  unguarded.
- **`pipefail` plus `timeout` produces a false negative.** `timeout 20 ros2 topic
  hz X | grep average || echo SILENT` prints the rate **and** "SILENT", because
  the timeout kills the producer and pipefail fails the pipeline. Capture first,
  match second.
- **No double hyphen inside an XML comment.** It is not legal XML.
- **Foreground `sleep` is blocked.** Sleep inside the container script instead.
- **`docker compose up -d` plus `docker exec`, never `compose run`**, which makes
  a new container every time. That is how six `gz sim` processes once ran at
  once.
- **Never two simulators on one ROS domain.** Both publish `/clock`, `/odom` and
  `/tf`, time jumps, and Nav2 plans against a teleporting robot. Run
  `sim_preflight.sh` first. Note its own `pgrep -f` self-matches an inline
  `bash -c` whose text contains the patterns, so confirm with `ps` if it looks
  wrong.
- **Count messages; never trust `real_time_factor`.** gz-sim runs physics and
  rendering on separate threads, so RTF sits near 1.0 while cameras deliver
  7 Hz.
- **`ros2 topic echo --once --field X` is unreliable** for this. It printed
  nothing for a topic that a real subscriber was receiving at 11.8 Hz. Write a
  subscriber node.
- **`sensor_msgs_py.point_cloud2.read_points` returns a STRUCTURED array** on
  Jazzy; index by field name. And **`skip_nans` does not drop infinities**: the
  depth is exact ray-cast range with `+inf` past the far clip, so filter with
  `np.isfinite` explicitly.
- **`docker compose down -v` after rebuilding an image**, or named volumes keep
  serving the old workspace.

---

## 9. This session's measurement scripts

Copied into `scripts/gazebo/measurements/` so the numbers in the report can be
reproduced. Each carries a header saying what it measures and how to run it.

| Script | Produced |
| --- | --- |
| `topic_counter.py` | The cross-container DDS gate counts. Run in both containers at once |
| `skew_probe.py` | RGB and depth stamp alignment: median gap 0.0000 s, synchroniser fired 396/396 |
| `torch_probe.py` | torch 2.14.0+cu130, `sm_120` present, a real convolution on the GPU |
| `world_vs_track_check.py` | World versus `track.png` corridor: median error 0.043 m, 90% within 0.15 m |
| `clearance_rate_sensitivity.sh` | The 2.4x sampling-rate artifact in lane clearance |

**Not copied, too rough to keep:** the one-off orchestration wrappers that
launched a sim, waited, ran one of the above and killed it
(`dds_gate.sh`, `p03.sh`, `p04b.sh`, `test_c2.sh`, `test_c3*.sh`,
`downsample` driver). They are three lines of `ros2 launch`, a `sleep`, and a
`pkill`, and they hardcode container paths. Rebuild them from the headers of the
scripts above rather than hunting for them.

---

## 10. What is next, in order

1. **Meeting prep: get `main` clonable for the sub-team tonight.** This outranks
   everything else and arrives as its own prompt. Note the freeze rule: all
   three gates must pass on whatever lands in `main`.
2. **Decide the `IN LANE` gate** (section 4 caveat 1). The regression baseline is
   unreliable until this is settled, and it is a one-line change once decided.
3. **Close P0-1 properly.** The `sequence size exceeds remaining buffer`
   deserialization error is an open defect, not a passed gate. Capture the
   failing packet or identify the field before trusting YOLOPv2 results from the
   Humble container.
4. **Fix the point-cloud frame (P0-3).** Three options are written up in the
   report; the republisher is the only one that does not disturb the perception
   path. **Re-baseline all three gates afterwards**, because enabling
   `obstacle_layer` connects a consumer that has never been connected.
5. **Score `lane_segmentation` (YOLOPv2).** Needs `fetch_yolopv2_weights.sh` run
   (no checksum, note it) and depends on 3.
6. **P1**: instrument the FollowPath abort count before tuning anything; publish
   `/fix`; write the RQ-06 and RQ-11 decision briefs; the runbooks for the other
   machines.
