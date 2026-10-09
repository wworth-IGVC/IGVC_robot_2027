# Gazebo simulator: what is done, and what is next

**Updated 2026-10-07.** A single page for "where is the simulator up to".
`GAZEBO_SETUP.md` is the detail behind every line here.

Priorities: **P0** blocks other work · **P1** needed before competition ·
**P2** worth doing · **TEAM** needs a decision, not an implementation.

---

## Done, and verified

Verified means a command was run and a number came back. Nothing below is
"should work".

### The simulator itself

- [x] **ROS 2 Jazzy + Gazebo Harmonic**, no source builds, no third-party apt
      repo. Jazzy runs to May 2029; Humble ends a month before the competition.
- [x] **GPU rendering proven**, not assumed. `D3D12 (NVIDIA RTX 5070 Ti
      Laptop)`. The default failure is a *silent* fall back to CPU, so
      `render_check.sh` exists and is the first thing anyone runs.
- [x] **GUI works from WSL2** via WSLg, which settled the dual-boot question:
      nobody installs a second OS.
- [x] **The IGVC course is generated** from `track_points.json`, the same file
      Isaac reads, so both simulators share one coordinate frame with no
      alignment step.
- [x] **The robot loads and drives.** Meshes resolve, sensors and plugins
      survive the URDF to SDF conversion.
- [x] **Four zero-normal collision meshes worked around** for simulation with
      primitive colliders under `sim:=true`. The real robot's description is
      untouched.

### ROS 2 Jazzy only, no Humble in the simulator path  *(2026-09-29)*

- [x] **Every package and robot component classified for Jazzy**,
      [`JAZZY_MIGRATION.md`](JAZZY_MIGRATION.md): 22 of 22 packages build,
      nothing needs rewriting because of Jazzy; the robot-side work (JetPack,
      the ZED stack, torch on the Orin) is laid out with the decisions it
      needs. `scripts/check_jazzy_inventory.py` fails if a package is missing
      from it.
- [x] **Every package declares what it uses.** A clean `rosdep install` now
      brings in `twist_stamper`, `joy` and `teleop_twist_joy`, and
      `rosdep check` exits 0; `igvc_test_bringup`'s scripts install where
      `ros2 run` finds them without `--symlink-install`.
- [x] **`igvc-dev-jazzy` replaced the Humble torch image**: the Gazebo image
      plus torch (CUDA 13.0, `sm_120`) and every workspace dependency. It
      runs as the simulator container with both gates passing, and YOLOPv2
      runs in it.
- [x] **Humble is out of the simulator path.** The Humble Gazebo image is
      deleted, the Humble torch image is in `docker/deprecated/` for one
      season with no compose service, and what remains Humble is robot side
      (the ZED images).
- [x] **The 2026 competition code runs in a container with a ROS 2 terminal
      on it**, from PowerShell: [`COMPETITION_STACK.md`](COMPETITION_STACK.md).
      34 nodes, the graph measured on 2026-09-24, Nav2 active, YOLOPv2 on the
      GPU.

### RQ-03, the interface contract  *(closed 2026-09-15)*

- [x] **The section 3.7 contract is one YAML file**,
      `config/gazebo_bridge.yaml`. The bridge renames Gazebo's topics onto the
      ZED namespace the 2026 stack already reads, so nothing downstream was
      modified. No shim node, no second serialisation of the image stream.
- [x] **Odometry frame corrected.** Gazebo's odom axes follow the spawn
      heading, 134.87 deg off world here, while the stack assumes world axes -
      and `gt_nav_bridge_node` reads odometry as a map-frame pose with no TF
      lookup, so no amount of remapping could have fixed it.
      `gazebo_odom_shim` rotates it and owns `odom -> base_link`.
- [x] **A frame-id typo fixed that failed silently.** The cameras stamped a TF
      frame that did not exist; `lane_detection.py` falls back to pinhole
      projection when the lookup fails and still draws plausible, wrong lane
      lines. Found by `RQ03_AUDIT.md`.

### Autonomy  *(the milestone)*

- [x] **The robot drives the course by itself.** Ground-truth lane grid ->
      navigator -> Nav2 -> `/cmd_vel`, all 2026 code, unmodified, using the
      real robot's `nav2_lane_follow_config.yaml`.
      **83.0 m driven under its own control, max 1.86 m off the lane
      centreline, and 0.0% of samples with a footprint corner over the painted
      line**, with the test publishing no velocity command at all
      (2026-09-17). The earlier figures of 122.9 m and 1.79 m came from a
      pose log sampled at about 2 Hz; see the clearance note below for what
      that rate hid.
- [x] **Nav2 is in the image** - `navigation2`, `nav2-bringup`,
      `image-geometry`, `twist-stamper`.

### Tests that can actually fail

- [x] `render_check.sh`, GPU or silent software fallback. **Was listed here
      while broken on a clean container; see the retractions below.** Fixed
      2026-09-17 and re-verified from a shell with nothing sourced: exit 0,
      `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`.
- [x] `bringup_smoke_test.sh`, **18 of 18, 0 failures.** Contract topics carry
      messages, camera frame ids resolve through TF, and odometry is compared
      against Gazebo's own ground truth in heading *and* distance. That last
      check catches a wrong odom frame and a wrong wheel radius at once.
- [x] `autonomy_check.sh`, does the robot drive the course, never touching
      `/cmd_vel`. **Now also reports lane-boundary clearance** *(2026-09-17)*:
      the gap between the rotated footprint and the inner edge of the painted
      line, at the **local** lane half-width, for both the 0.810 x 0.970 m
      chassis and Nav2's 0.700 x 1.000 m footprint. Reported, deliberately not
      a pass criterion, because which footprint is right is still a team
      question. Yaw matters and is not a refinement: a rectangle reaches
      0.405 m sideways when aligned with the lane and up to 0.632 m when
      skewed, at 50.1 degrees, and the robot sits at a **mean 20.7 degrees**
      to the lane with a peak of 65.3, because the navigator is continuously
      re-aiming. Pose sampling was raised from 2 Hz to 10 Hz so a short
      overshoot cannot fall between samples.
      **The margin is 8 to 9 cm, not the 25 cm first reported.** Three runs
      with the rotated footprint at honest sampling: worst clearance +0.093,
      +0.087 and +0.078 m against the chassis, 0.0% of samples over the line
      in every run. A fourth run today gave **+0.040 m**. So the milestone
      claim "never left the course" **stands, and is now measured against the
      painted boundary** rather than inferred from a centreline distance, but
      it is a much less comfortable number than the earlier unrotated,
      under-sampled version of this metric gave. Downsampling one 831-sample
      run shows why: at 2 Hz the same run reports +0.227 m. **The worst
      clearance is a brief event and the worst deviation is not**, so the old
      rate overstated the clearance margin by at least 2.4x while leaving the
      deviation untouched.
- [x] `sim_preflight.sh`, refuses to start a second simulator on top of a
      first, because two on one ROS domain produce results that look real and
      are worthless.
- [x] **Barrel contacts are counted** *(2026-10-01)*. Every barrel carries a
      contact sensor; `autonomy_check.sh` reports each touch (barrel, robot
      part, simulation time) and prints NOT MEASURED rather than "none" when
      the chain is not live. `measurements/barrel_contact_check.sh` drives
      into barrel 0 (1175 contact messages) and down an open lane (0).
      **`BARRELS` is a pass criterion**: a touch fails, and so does being
      unable to measure one, because a touch voids every other reading (they
      come from odometry, which kept counting while a pinned robot's wheels
      slipped: a "normal lap" while ground truth put the robot 14.9 m from
      where odometry said). First default run: **no contact**, so the
      ground-truth robot's barrel avoidance is now measured, not inferred.
      `GAZEBO_SETUP.md` 10.11.

### Documentation

- [x] `docs/setup/WINDOWS.md` + `docs/setup/IGVC_2027_Windows_Setup_Guide.docx`, zero to a
      driving robot, including a section for machines unlike the one this was
      built on.
- [x] `GAZEBO_SETUP.md` sections 9 and 10; `DOCKER_CHANGES.md` section 13.
- [x] `igvc_gazebo_linux` service so native Linux has something to run.

### The world is the course the planner plans in  *(2026-09-17)*

- [x] **The painted lane is variable, 10 to 20 ft, not a constant 12 ft.**
      `IGVC_track_generator/constants.py` sets the range and `main.py:503`
      paints it as a sinusoid, two cycles per lap, straight out of IGVC rules
      II.2. `track_ground_truth_node` reads that from `track.png`, so it is the
      corridor Nav2 plans in, while the world generator painted a constant
      3.658 m. **Measured against `track.png` at 200 points: median corridor
      error went from -0.813 m to -0.043 m, and agreement within 0.15 m from
      7.0% of samples to 90.0%.** The constant is still available as
      `--lane-width-ft` for a debug course.
- [x] **The width profile is imported from the track generator, not copied.**
      `generate_igvc_world.py` reads `constants.py` directly and fails loudly
      if the submodule is missing. Verified: the imported values regenerate a
      byte-identical world.

### Every machine, and a native Mac route  *(2026-09-24)*

- [x] **The checks no longer depend on how fast the machine is.** Fixed
      `sleep 35` / `sleep 50` became readiness waits (`wait_ready.py`), the
      autonomy watch window is 100 s of **simulation** time, and the smoke
      test's drive runs on simulation time in one process
      (`drive_probe.py`). Before this, the same code drove 83.0 m one day and
      47.4 m the next because the simulator ran at half real time; now
      89.9 m on tier A and 89.2 m on software rendering at 0.29x, each over
      exactly 100.0 s of sim time. **Autonomy distances from before
      2026-09-24 are not comparable with later ones.**
- [x] **The smoke test no longer drives off the slab.** It did 2 runs in 2 on
      2026-09-24, because the robot drove on a held command for as long as
      four CLI pose samples took to start. Travel is now bounded at 3.2 m by
      construction; measured 2.759 and 2.761 m.
- [x] **`ON THE SLAB` no longer passes unmeasured.** A failed height read
      used to default to the spawn height.
- [x] **A native macOS route: pixi + RoboStack**, no Docker, Metal rendering
      and a Gazebo window expected. Resolves for Apple Silicon (969
      packages); passes bootstrap 18/18 and autonomy in a clean Linux
      container on the same recipes; peak memory 3.5 GB.
      `scripts/gazebo/native_dryrun.sh` reruns that proof in one command.
- [x] **The repo says which path is yours.** `SETUP.md` at the root,
      platform guides in `docs/setup/`, maintainer steps in
      `docs/MAINTAINING_ENVIRONMENTS.md`.
- [ ] **A Mac owner runs `docs/setup/MACOS_CHECKLIST.md`.** Nothing has run
      on a Mac. Steps 7 (Metal, tier M) and 10 (the Gazebo window) are the two
      only a Mac can answer.
- [ ] **Does the simulator clock stall recur?** Seen once on tier A on
      2026-09-24: `/clock` stopped about 2 s in, every camera topic silent.
      Hypothesis: a first render that never completes, holding the
      sensors-gated physics step. See the report, section 19.6.

### Retracted, on purpose

- [x] **"No `/cmd_vel` timeout"**, withdrawn. The robot had driven off the
      ground slab and was falling with its wheels spinning, odometry counting
      up as though it were driving.
- [x] **"It circles at barrels"**, withdrawn. Six `gz sim` processes were
      running at once.
- [x] **`render_check.sh` was listed as done and verified, and failed on a
      clean container**, withdrawn 2026-09-17. It never sourced ROS, and `gz`
      is not on PATH until you do, because on this image Gazebo comes from the
      ROS vendor packages. The documented first command on a new machine
      printed `gz: No such file or directory` and `RESULT: UNKNOWN`, which
      reads as a broken GPU rather than a missing PATH entry. Fixed.
- [x] **"Four FollowPath aborts in 143 s"**, withdrawn 2026-09-17. That number
      is `/navigator/status`'s `aborts=` field, which is
      `min(consecutive + 1, 4)` at `navigator.py:1595`, `:1644` and `:1684`,
      **reset to zero on success** at `:1675`, and incremented on goal
      *rejection* as well. It saturates at 4 and cannot distinguish 4 from 40.
      `autonomy_check.sh` samples it once at the end and never counted aborts.
      The true total is unknown; instrument it before tuning anything.
- [x] **"`lane_detection_node` cannot start without torch"**, withdrawn.
      `lane_detection.py` imports no torch; it is CLAHE, Canny and
      `HoughLinesP`. The node that needs torch is `lane_segmentation_node`.
- [x] **"The synchroniser never fires and logs nothing"**, withdrawn.
      `lane_detection.py:81-91` builds one synchroniser **per camera**, and
      `:119` plus `:924-929` install a 2 s watchdog that warns. Measured: with
      the front camera alone, the RGB and depth streams are aligned to a median
      stamp gap of **0.0000 s** and max 0.066 s, and the synchroniser as built
      at `:88-89` fired **396 times in 396 message pairs**.
- [x] **"The robot is 0.70 m wide"**, withdrawn. That is the Nav2 footprint
      parameter. The chassis collision mesh is **0.810 x 0.970 m**, and the
      chassis is the widest part, wider than the wheels.

---

## Not done

### After the merge of the competition branch  *(2026-10-08, `GAZEBO_SETUP.md` 10.15)*

- [x] **DONE:** `67a6934` merged on branch `adopt-competition`; every package
      builds; render, smoke and autonomy pass three times (89.0 to 90.1 m, `LANE`
      clearance +0.120 to +0.173 m). Moving `main` is Liam's.
- [ ] **The competition navigator's path-heading clamp and validator disagree at
      the boundary** (`navigator.py` `_limit_path_heading` and
      `_path_invalid_reason`): a lane that turns more than `max_path_heading_rad`
      (43 degrees) inside the look-ahead leaves the robot holding and reversing.
      The simulator hides it with `max_path_heading_rad: 1.57` in
      `gazebo_nav_test.launch.py`. Fixing it needs a unit test (a path at
      exactly the limit must be accepted), a decision on the number, and Liam's
      approval (protected package). Not started.
- [ ] **`ublox_gps` needs `libasio-dev`, which the simulator image lacks.** An
      unchanged submodule, so `main` has the same property; the gates do not
      build it. Add it to `docker/Dockerfile.gazebo-jazzy` only if the team wants
      "build every package" to work in that image.
- [ ] **Obstacles from the lidar map are in the loop but cross the paint**
      (`obstacle_source:=lidar_map`, 2026-10-08, `GAZEBO_SETUP.md` 10.16): 3 of 3
      runs touch no barrel (the `none` control pins on barrel 4), and 3 of 3 FAIL
      `LANE` (-0.144 to -0.696 m), each squeezing between barrel 4 and the right
      line. Next: separate the causes listed there (inflation, decay, line cost);
      any fix is competition code or parameters, so it needs Liam's approval.
- [ ] **The robot USD and the URDF now disagree further** (new front ZED 2i link,
      lidar and mast, side cameras at 45 degrees; research pass 3, RQ-09). Which
      is the real robot is a tape-measure question for Aidan.

### P0, blocking the merge of `jazzy-only` into `main`

- [x] **Done 2026-09-29 evening: the freeze rule is met** (report section
      20.9.1). On the final tree, in both `igvc_gazebo` and `igvc_dev_jazzy`,
      one simulator at a time with nothing else running in the container:
      render PASS, smoke 18/18 three times, autonomy PASS three times, no
      leftovers. The two bad runs below did not recur; each is recorded as
      seen once, not explained. A slow simulator does not reproduce the FAIL:
      the check passes with Gazebo held at 0.2x real time, and with the
      container starved to 2 CPUs (about 0.21x). The aborted run's own files
      show `ros2 launch` died first while the nodes and Gazebo ran on, so
      "every process vanished" was the check's teardown afterwards.
- [x] ~~**The Gazebo image's autonomy check did not pass on the final tree,
      2026-09-29, and nobody knows why.**~~ Superseded by the item above; the
      original entry is kept below. Two runs in `igvc_gazebo`, the
      unchanged simulator image:
      1. **FAIL** while the laptop's GPU power setting held the GPU in P8:
         the simulator ran at about 0.21x real time and the robot left the
         lane, 7.87 m off the centreline, mean yaw to the lane 98.6 deg,
         31.6% of samples over the paint, although it drove 87.5 m.
      2. After the setting was raised: **aborted**, because every simulator
         process vanished at 46 s of simulation time. The launch log has no
         shutdown lines, the container's cgroup shows no out-of-memory kill,
         and Docker's event log shows no kill command.

      The same check passed in this image on 2026-09-24 before the branch
      changed anything, and passed twice today in `igvc-dev-jazzy`, which is
      this image plus torch. The branch changes only what the checks run
      after they finish (`sim_teardown`). **Not explained, and two bad runs
      in a row is not "seen once".** Next: rerun it three times on a quiet
      machine with the GPU power setting high, keeping each run's
      `/tmp/track_log.txt` (the failed run's was overwritten), and watch for
      the vanishing processes with `ps` while it runs. Until three passes in
      each image are recorded, the freeze rule is not met and `jazzy-only`
      must not be merged. Report section 20.9 has the logs.
- [ ] **A slow GPU power setting slows the whole simulator, silently.** With
      the GPU held in P8 (about 250 to 700 MHz) the simulator's real-time
      factor fell to 0.21, against 0.4 on 2026-09-24 and 0.56 earlier the same
      day, and `render_check.sh` still passed, because the GPU was in use,
      just slowly. Worth a line in the Windows guide once it is understood,
      and possibly a real-time-factor warning in the checks. **Downgraded
      2026-09-29 evening: slow is not a failure.** The autonomy check passes
      at 0.2x (report 20.9.1), and the GPU sat in P8 during the evening's
      passing gates too, which ran at about 1.0x. Now P2: a guide note only.

### P0, blocking real perception work

- [x] **Hough lane detection already runs, in the Gazebo image, today.**
      *(2026-09-17)* `lane_detection.py` imports no torch, and the Gazebo image
      already carries `cv_bridge`, `message_filters`, `image_geometry`, numpy
      and OpenCV. Measured with the stock config and the robot driving itself
      for 90 s on the front camera alone: **`/lane_map` 253 occupied cells of
      102400, `/lane_costmap` 800 of 160000**, one startup watchdog warning,
      no errors. This is the cheap half of perception and it needed nothing.
- [x] **YOLOPv2 runs against the simulator, on Jazzy, and has a number.**
      *(2026-09-29)* In the Jazzy dev image `igvc-dev-jazzy`, torch 2.14.0 on
      `cuda:0` in half precision, weights checked against a recorded SHA256
      by `fetch_yolopv2_weights.sh`. Scored by `lane_eval_sim.py` in one
      self-driving run, 100 s, front camera, 640x360, with Hough and the
      ground truth scored in the same drive:

      | Producer | Hit rate @ 0.25 m | Occupied cells | Coverage near observed cells |
      | --- | --- | --- | --- |
      | **YOLOPv2** | **0.166** | 2721 | 0.762 |
      | Hough, same drive | 0.389 | 18 | 0.273 |
      | ground truth (the ceiling) | 0.676 | 2946 | 0.996 |

      **Not good enough to steer by yet**: it finds most of the paint it can
      see but places it with a median error of 0.77 m. No pass threshold was
      set; this is the baseline. Causes not yet separated: the 640x360
      resolution (its thresholds were tuned at 2.4x that), the unknown lens,
      and its depth projection. **This Hough number is not a re-baseline:**
      in this run Hough found no TF from the camera's optical frame at startup
      and fell back to pinhole projection for the whole run (13 to 16 points a
      frame, 18 cells), so compare YOLOPv2 with the 2026-09-17 Hough figure,
      0.491, instead.
- [x] **`num_cameras: 3` against one camera is a startup warning, not a
      failure.** *(2026-09-17)* Two separate claims were wrong: the node builds
      one synchroniser **per camera**, so camera 0 is unaffected, and it does
      log. Evidence above: 253 occupied cells with `sim_cameras:=front` and the
      stock `num_cameras: 3`. **No sim override is needed for function**; the
      target stays `sim_cameras:=all` for a three-camera budget measurement.
- [x] **The camera point cloud IS rotated 90 degrees. Answered 2026-09-17, and
      it is the bad answer.** The cloud carries **body**-convention data, x
      forward, stamped with the **optical** frame. `gz_frame_id` sets a header
      string; it does not rotate data.

      | Run | Normal axis | Offset | Angle to +Z | Inliers |
      | --- | --- | --- | --- | --- |
      | Through the cloud's own stamped frame | **X** | -0.0994 m | **89.78 deg** | 100% |
      | Forced through the body frame (control) | Z | -0.2311 m | 0.39 deg | 100% |

      Predicted from the URDF *before* measuring: -0.096 and -0.229 m.
      Measured: -0.0994 and -0.2311. **Agreement to 3.4 mm and 2.1 mm**, so
      the fix is a known quantity rather than a guess. Through the stamped
      frame the whole cloud collapses into a 12 cm slab, which is the floor
      stood on its end. Check committed as
      `scripts/gazebo/cloud_frame_check.py`; it names which of four mistakes
      was made rather than just failing.
      One trap worth keeping: `skip_nans` does **not** drop infinities, and
      the bridge passes exact ray-cast range with `+inf` past the far clip.
      The first run reported `inf` extents and 45% inliers. Filter with
      `np.isfinite` explicitly. Also, `read_points` returns a **structured**
      array on Jazzy; index by field name.
- [x] **The 2D lidar sees the barrels, in Nav2's frame, today.**
      *(2026-09-24)* Robot parked 2.77 m from a barrel at a 45 degree yaw:
      100.0% of 480 returns beyond the footprint land on a barrel through TF
      in `odom`, barrels at 2.77, 5.80 and 8.89 m all hit, and both negative
      controls (no transform; spawn yaw applied twice) score 0%. The scan
      plane is about 0.69 m up and the barrels are 0.9 m tall.
      `scripts/gazebo/measurements/lidar_barrel_check.py`. **So obstacle
      perception does not have to wait for the point-cloud fix below:**
      `obstacle_layer` with `lidar_scan` as its only source is the smaller
      first step. It is still a consumer that has never been connected, so
      re-baseline all three gates when it is. The lidar cannot see paint, so
      lanes stay a camera problem; 80 own-body returns at 0.10 to 0.13 m are
      inside the footprint the layer clears.
- [x] **Measured 2026-10-01: Nav2's obstacle layer on `/scan` does NOT make
      `main`'s robot avoid a barrel**, which retracts "the smaller first
      step" above. With the barrels removed from the planner's grid
      (`obstacle_source:=none`) the robot drove into barrel 4 and stayed
      pinned, 2 runs of 2; with the layer added (`obstacle_source:=nav2_scan`)
      exactly the same, 2 of 2, although the layer loaded and subscribed.
      Nothing that moves the robot reads Nav2's costmaps: the navigator plans
      on `/lane_costmap` alone and drives FollowPath, RPP collision detection
      is off, the collision monitor has no source. Separately, the lane layer
      hides the obstacle layer's marks in the published costmaps (mechanism
      not established). `GAZEBO_SETUP.md` 10.12.
- [ ] **TEAM: bring the competition branch's lidar obstacle map to `main`.**
      `lidar_obstacle_costmap.py` and the navigator overlay of its grid
      (`2026/more_diverging_changes`, `navigator.py:1270-1295`) are how the
      2026 robot avoided obstacles in lane mode. Both are in the protected
      `igvc_lane_detection`. Once agreed: `obstacle_source:=lidar_map`, judged
      by `autonomy_check.sh` (0 barrel contacts, three runs) with
      `obstacle_source:=none` as the negative control.
- [x] **DONE 2026-10-07: the point-cloud frame is fixed and the cloud arrives.**
      `gazebo_cloud_restamp` republishes the cloud in the camera's body frame
      (3 of 3 frame arms as predicted), and a Fast DDS flow controller on the
      cloud writers (`config/fastdds_sim_camera_pacing.xml`) makes a best-effort
      subscriber receive it: 680 clouds against 680 camera frames in 60 s,
      where unpaced it saw 15% to 34% of the camera's frames. The loss was never the Python node, it
      was UDP datagrams dropped from a 3.7 MB burst. `scripts/gazebo/
      cloud_rate_check.py` is the check; `GAZEBO_SETUP.md` 10.13 has the
      diagnosis, the controls and what is not known. The original item
      follows, kept for its reasoning.
- [ ] **(superseded, see above) Fix the point-cloud frame, and expect it to change behaviour.**
      *(2026-10-01: on `main` it most likely will not, because nothing that
      moves the robot reads Nav2's costmaps, measured in the item above; the
      competition branch's YOLOPv2, which projects lane pixels through this
      cloud, is what needs it. And the "three options written up in the
      report" were never written: no branch has them. The fix is a re-stamp
      of the cloud alone to the camera's body frame, because `gz_frame_id`
      also stamps the image, depth and camera_info, which must stay
      optical.)* Three
      options are written up in the report; a republisher is the only one that
      does not disturb the perception path. **This is not a bug fix, it is a
      new system under test.** `gazebo_nav_test_nav2_overrides.yaml:36-44`
      replaces `plugins` with `["lane_layer", "inflation_layer"]` in both
      costmaps, so `obstacle_layer` is never instantiated and the clouds have
      **never** reached it. `autonomy_check.sh` has therefore been passing
      with no live obstacle evidence whatsoever. Fixing the frame and
      re-enabling the layer connects a consumer that has never been connected.
      **Re-baseline all three gates afterwards** and do not attribute the
      change to the frame fix alone. Deliberately **not** merged for the
      2026-09-17 sub-team meeting for exactly this reason.
- [x] **P0-1 closed by removal, 2026-09-29.** Perception now runs in the
      Jazzy dev image `igvc-dev-jazzy`, so no Humble participant is left on
      the simulator's graph. `scripts/gazebo/measurements/dds_idle_probe.py`,
      a node that subscribes to nothing, 60 s beside the running simulator:
      **0 lines from the Jazzy dev container** (with and without the Fast DDS
      profile), **3 lines from the Humble container** (with and without), and
      0 from the Humble container with no simulator up, so the test still
      fails where it should. A new clue settles most of the mechanism: the
      Humble probe **discovered no nodes at all** while the Jazzy one found 4,
      and `rmw_dds_common`'s `Gid` is `char[24]` on Humble against `char[16]`
      on Jazzy (read from both images). Node names travel in
      `/ros_discovery_info` messages made of Gids, so a Humble reader
      misparses Jazzy's, prints the line, and never learns a node name, while
      topic data still flows because DDS endpoint discovery does not use that
      topic. That link is inferred; the sizes and the zero are measured. The
      original entry follows, unchanged.
- [x] ~~**Close P0-1 properly: a partial pass with an open defect.**~~ The
      cross-container DDS gate passed on counts. Over 30 s, counting in both
      containers simultaneously, Jazzy and Humble agreed within 1% on every
      topic YOLOPv2 consumes: rgb 354/356, depth 353/355, camera_info 357/357,
      zed odom 691/691, `/odom` 691/691, `/clock` 17188/17011, `/tf`
      1161/1159. Payloads are right too: 691200 B = 640x360x3 RGB, 921600 B =
      640x360x4 32FC1 depth, and `/tf_static` delivers its one latched message
      with 24 transforms once the subscriber uses `TRANSIENT_LOCAL`.
      **But the Humble container prints `sequence size exceeds remaining
      buffer`, verbatim, exactly three times per process.** It is not
      topic-specific (same three lines whichever single topic is subscribed),
      not data-related (a Humble node subscribing to *nothing* still prints
      all three), and **directional** (a Jazzy node subscribing to nothing
      prints none). Working hypothesis: a discovery-phase parse failure where
      Humble's Fast DDS cannot read something Jazzy's participants advertise.
      **That is a hypothesis.** The failing packet was not captured and the
      field was not identified, and it was not ruled out that some low-rate
      message is dropped silently in a way 30 s of counting would not reveal.
      Counts staying intact while a deserializer complains is exactly the
      silent-partial-failure class that has bitten this project twice. **Do
      not trust YOLOPv2 results from the Humble container until this is
      closed.**
- [x] **A sim-side lane evaluator exists, and produced the project's first
      non-zero lane numbers.** *(2026-09-17)* The existing `lane_eval_node`
      cannot score by construction: it thresholds both grids at 50, but
      `track_ground_truth_node` writes 0 for the free corridor and -1
      elsewhere and never writes anything at or above 50, while
      `lane_detection` writes 100 for lane lines. The two are complementary
      encodings of different things, a filled corridor against painted lines,
      so ground truth is always all-False and IoU is always 0.0. It has scored
      identically zero since the encoding changed on 2026-05-14 at 11:12, five
      hours after the file first appeared.
      `scripts/gazebo/lane_eval_sim.py` instead scores a detector against lane
      **lines** generated from the same `track_points.json` and width profile
      the world is painted from, so reference and stimulus are the same
      geometry by construction. Front camera only, robot self-driving 100 s,
      tolerance 0.25 m:

      | | Hough, own grid | control: the ground-truth bridge's grid |
      | --- | --- | --- |
      | cell IoU | 0.0434 | 0.2314 |
      | hit rate @ 0.25 m | **0.491** | 0.676 |
      | coverage, observed region | 0.464 | 0.996 |
      | predicted to paint, median | 0.255 m | 0.096 m |

      **Read these against the control, not against 1.0.** The control is
      derived from the same JSON as the reference and still only reaches
      0.676, because `gt_nav_bridge_node` stamps a lethal rim near but not
      exactly on the painted line. So **0.676 is the practical ceiling and the
      Hough detector's 0.491 is about 73% of achievable.**
      A measurement error caught before reporting: the first run scored 0.2314
      and it was the ground truth scored against itself, because
      `lane_detection.py` hardcodes `/lane_map` and `gazebo_nav_test.launch.py`
      also starts `gt_nav_bridge_node` publishing the same name. Caught by
      checking the grid geometry against the config rather than trusting the
      topic name. The rerun remaps to `/hough/lane_map`.
- [x] **DONE 2026-10-07 (decision D4): `IN LANE` is now `LANE`.**
      `autonomy_check.sh` used to compare centreline deviation against a flat
      **2.00 m**, which failed runs that never touched the paint (2.04 m on
      09-17 and 2.07 m on 10-01, both 0.0% over the line) because the lane is
      10 to 20 ft wide.
      `LANE` fails if the chassis footprint crosses the painted boundary at
      any sample, or if the clearance is not measured; the centreline number
      is reported only. Tested: negative controls fail, 18 saved runs
      re-scored with 0 surprises, one live run PASS. `ANALYSE_ONLY=<track_log>`
      re-scores a saved run without a simulator. `docs/GAZEBO_SETUP.md` 10.9.
- [ ] **The `LANE` margin is thin: 1.4 to 8.4 cm over 13 ground-truth runs
      (today's live run was +0.014 m), and the metric is good to about 3 cm.**
      A FAIL within about 3 cm of zero is noise until shown otherwise. The fix
      is steering that keeps further from the paint (navigator, Nav2: protected,
      not started), never a looser gate. Not urgent while the gate passes.

### P1, before competition

- [ ] **TEAM: decide RQ-06, the control path.** `ros2_control` is not in the
      loop at all - Nav2's twist goes straight to Gazebo's DiffDrive, so
      `/isaac_joint_cmd` is unbridged and joint limits, the controller update
      rate and its interaction with the physics step are untested. Choice is
      `gz_ros2_control` in-process versus a `GazeboDriveHardware` mirroring
      `IsaacDriveHardware` over topics. Real control-loop-timing consequences.
- [ ] **Count the FollowPath aborts before tuning anything.** The "four in
      143 s" figure is retracted above: it is a saturating gauge, not a count.
      Nothing currently counts aborts. Instrument `STATUS_ABORTED` results from
      the log, with the pose and the nearest obstacle at each, then compare
      across at least 3 runs per condition, because single runs have misled
      this project twice.
      The corridor hypothesis is still worth testing but the arithmetic behind
      it was wrong twice over, and the "0.70 m robot" is the Nav2 footprint,
      not the robot. Corrected: at the tightest barrel there is about 1.367 m
      of free cells, so a 0.810 m chassis **fits**, with roughly 0.557 m to
      spare. What does hold is that Nav2's `inflation_radius: 0.75` leaves
      **no zero-cost cell anywhere in that gap**, which is a cost-shape
      problem rather than a geometry one. Treat it as a hypothesis, not a
      diagnosis.
      **An earlier edition of this line said "`GAZEBO_SETUP.md` 10.4 already
      retracts its own 0.87 m gap". It did not.** That section still asserted
      both the 0.87 m gap and the 0.70 m robot until 2026-09-17, when it was
      rewritten. Do not trust a forward reference to a retraction without
      opening the file.
- [ ] **Run `render_check.sh` on the other machines.** The RTX 5080 Laptop and
      the Windows 10 machine are still untested, and the Intel-adapter crash
      depends on which GPUs a laptop has.
- [ ] **Verify `igvc_gazebo_linux` on a real Linux machine.** Written, never
      run. Delete the caveat in `DOCKER_CHANGES.md` 13.4 when it works.
- [x] **RQ-11 decided 2026-10-01 (Liam): barrels are static and solid, not
      knockable.** The rules penalise any touch and end the run on a crash or
      a displaced barrel, so the simulator only needs to know whether the
      robot touched one, and a barrel that could move would change the course
      mid-run. Isaac's generated field still gives them **no collision at
      all**; `--no-barrel-collision` reproduces that. Touches are now counted
      (`GAZEBO_SETUP.md` 10.11).
- [x] **`/fix` is published** *(2026-10-07)*: `<spherical_coordinates>` and
      the NavSat system in the world, a `navsat` sensor on
      `front_gps_antenna_link`, bridged as NavSatFix. `measurements/fix_check.py`
      PASS (0.002 m, 0.00 deg); its control with the axes 90 deg off FAILS
      (18 to 20 m, 90 deg). **The datum is a placeholder**, and nothing in the
      sim consumes `/fix` yet. `GAZEBO_SETUP.md` 10.14.
- [ ] **TEAM (Aidan): the course datum, and `/fix` against `/gps/fix`.** No
      surveyed IGVC course coordinate exists in the repository; and `main`'s
      `localization.py` and `navigator.py` read `/gps/fix` while the u-blox
      driver publishes `/fix` (the competition branch uses `/fix`).

### P2, worth doing

- [x] **`/cmd_vel` timeout: SETTLED 2026-09-24. DiffDrive holds the last
      command indefinitely**, 1.605 m in 4.01 s of sim time with no command,
      still at 0.400 m/s, measured between stamped odometry messages by
      `drive_probe.py`. The real robot's controller times out, so sim is the
      less safe of the two. The text below is the history.
- [x] ~~Settle the `/cmd_vel` timeout properly.~~ Asserted and withdrawn
      twice. Needs a test that timestamps the last command against the last
      odometry change rather than inferring a window from `ros2 topic echo`
      latency. `diff_drive_controller` does time out, so a difference runs in
      the unsafe direction.
- [ ] **Repair the four zero-normal collision meshes**, or decide the
      primitive colliders are permanent for simulation. They are still broken
      for Isaac and MoveIt.
- [x] ~~**Run `sim_cameras:=all` against the real course.**~~ **Not needed,
      2026-10-01: the team is moving to one camera, the centre (front) one**
      (Liam), which is already the default `sim_cameras:=front`. The
      three-camera option stays in the description for the 2026 robot.
- [ ] **Does YOLOPv2 tolerate 640x360?** The camera budget forces that
      resolution. If perception needs 720p, three cameras are not viable on
      this hardware - which is also the AGX-versus-Orin-Nano argument.
      *(2026-09-29: it runs at 640x360 and scores 0.166 against a 0.676
      ceiling; whether resolution is the cause is not separated yet. A run at
      1280x720 on the same drive would answer it.)*
- [ ] **`wheel_radius`, for the real robot.** The simulator now corroborates
      0.1016 rather than the 0.2032 in `controllers.yaml` - odometry matched
      ground truth to 0.3% across four runs, and 0.2032 would give a ratio near
      2.0. Somebody should still put a tape measure on the wheel.

### Deliberately out of scope for now

- [ ] **Isaac Sim parity and the bidirectional converter.** Still the project's
      stated goal; the Gazebo side had to work first.
- [ ] **The v4 AI/ML build spec.** Deferred until parity is reached.

---

## The shortest path to "it perceives"

Steps 1, 2 and 4 of the original four-step plan are done or dissolved. What is
left is narrower and harder than the original list implied.

1. ~~Fix `num_cameras` for the sim.~~ **Not needed.** One synchroniser per
   camera, so camera 0 is unaffected; 253 occupied cells with
   `sim_cameras:=front` against the stock `num_cameras: 3`.
2. ~~Verify the point cloud orientation.~~ **Done, and it is rotated 90
   degrees.** Fixing it is now its own item above, because it connects a Nav2
   consumer that has never been connected.
3. ~~Close the `sequence size exceeds remaining buffer` defect, then run
   `lane_segmentation_node` out of `igvc-humble-fused-drive`.~~ **Done the
   other way round, 2026-09-29:** perception moved to the Jazzy dev image, so
   there is no Humble participant left to hit the defect, and YOLOPv2 ran
   against the simulator from there, weights checksummed. Hit rate 0.166
   against a 0.676 ceiling (the P0 item above). The next step is making that
   number better, not getting it.
4. ~~Compare its lane output against `/lane_ground_truth`.~~ **That comparison
   does not work as stated** and the reason is worth keeping: the two grids
   are complementary encodings of different things, a filled corridor against
   painted lines, which are near-disjoint sets by construction. A perfect
   detector scores near zero against it. `lane_eval_sim.py` is the
   replacement, it works, and the Hough baseline to beat is a **0.491 hit
   rate against a 0.676 ceiling**.
5. **Judge YOLOPv2 by its lane-point output, never by the node staying
   alive.** `yolopv2_infer.py` resizes every frame to 1280x720 before
   letterboxing, so a 640x360 Gazebo frame is interpolated up and the tensor
   shape is identical to the real path. **It cannot crash.** Separately, its
   pixel-denominated thresholds were tuned at 2.4x the linear resolution, so
   area thresholds are **5.76x** stricter at 640x360 and linear ones 2.4x.

## Meeting preparation, 2026-09-17

- [x] **All three gates pass on `main`.** `render_check.sh` exit 0 with
      `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`;
      `bringup_smoke_test.sh` **18 passed, 0 failed**, odom 7.073 m against
      ground truth 7.081 m, heading 0.13 deg, ratio 0.9988;
      `autonomy_check.sh` **PASS**, 83.0 m driven, max 1.86 m off the
      centreline, worst clearance +0.040 m, 0.0% over the paint.
- [x] **One command from a fresh clone to a verified simulator.**
      `scripts/gazebo/bootstrap.sh`, run from a WSL2 shell. Prints PASS or
      FAIL per step and stops at the first failure. It pulls the image from
      GHCR and it reports the machine's GPU tier rather than assuming one.
      `BUILD_IMAGE=1` is the fallback when GHCR is unreachable.
- [x] **The image is published to GHCR**, `ghcr.io/wworth-igvc/igvc-gazebo-jazzy`,
      with a dated tag and `latest`, replacing the USB drive. Audited before
      publishing: no credentials, no SSH keys, no ZED SDK, no private registry,
      and **no project source baked in at all**, since the repo is
      bind-mounted at runtime. The LAN tarball path is kept as a documented
      fallback only, in the session runbook.
- [x] **Three GPU tiers, and `render_check.sh` now names yours.** It used to
      grep only for `nvidia|geforce|rtx`, so a machine doing genuine hardware
      rendering on an Intel or AMD adapter was reported `UNKNOWN` and read as
      broken. Measured with the GPU disabled, tier C passes
      `bringup_smoke_test.sh` at **18 of 18**, ratio 0.9982, at **0.854 real
      time** with cameras at **4.27 Hz** on 320x180 at 5 Hz. At the tier A
      default of 640x360 at 15 Hz it manages only 1.63 Hz and 0.608 real time,
      so the small configuration is the tier C recommendation. Cameras off
      returns real time to 0.996, which locates the whole cost in the camera.
- [x] **`setup-windows.ps1` no longer builds a 14 GB image the simulator does
      not use**, no longer recommends `compose run`, and prints both the disk
      it wants and the disk it found.
- [ ] **Run the whole path on the other machines.** The RTX 5080 laptop, the
      Windows 10 machine and native Linux are all still untested, and the
      quickstart says so at the top and the bottom.
