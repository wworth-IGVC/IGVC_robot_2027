# Gazebo simulator: what is done, and what is next

**Updated 2026-09-15.** A single page for "where is the simulator up to".
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
      **122.9 m driven, max 1.79 m off the lane centreline, never left the
      course**, with the test publishing no velocity command at all.
- [x] **Nav2 is in the image** - `navigation2`, `nav2-bringup`,
      `image-geometry`, `twist-stamper`.

### Tests that can actually fail

- [x] `render_check.sh` — GPU or silent software fallback. **Was listed here
      while broken on a clean container; see the retractions below.** Fixed
      2026-09-17 and re-verified from a shell with nothing sourced: exit 0,
      `D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`.
- [x] `bringup_smoke_test.sh` — **18 of 18, 0 failures.** Contract topics carry
      messages, camera frame ids resolve through TF, and odometry is compared
      against Gazebo's own ground truth in heading *and* distance. That last
      check catches a wrong odom frame and a wrong wheel radius at once.
- [x] `autonomy_check.sh` — does the robot drive the course, never touching
      `/cmd_vel`. **Now also reports lane-boundary clearance** *(2026-09-17)*:
      the gap between the rotated footprint and the inner edge of the painted
      line, at the **local** lane half-width, for both the 0.810 x 0.970 m
      chassis and Nav2's 0.700 x 1.000 m footprint. Reported, deliberately not
      a pass criterion, because which footprint is right is still a team
      question. Yaw matters and is not a refinement: a rectangle reaches
      0.405 m sideways when aligned with the lane and up to 0.632 m when
      skewed. Pose sampling was raised from 2 Hz to 10 Hz so a short overshoot
      cannot fall between samples.
- [x] `sim_preflight.sh` — refuses to start a second simulator on top of a
      first, because two on one ROS domain produce results that look real and
      are worthless.

### Documentation

- [x] `GAZEBO_QUICKSTART.md` + `IGVC_2027_Gazebo_Setup_Guide.docx` — zero to a
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

### Retracted, on purpose

- [x] **"No `/cmd_vel` timeout"** — withdrawn. The robot had driven off the
      ground slab and was falling with its wheels spinning, odometry counting
      up as though it were driving.
- [x] **"It circles at barrels"** — withdrawn. Six `gz sim` processes were
      running at once.
- [x] **`render_check.sh` was listed as done and verified, and failed on a
      clean container** — withdrawn 2026-09-17. It never sourced ROS, and `gz`
      is not on PATH until you do, because on this image Gazebo comes from the
      ROS vendor packages. The documented first command on a new machine
      printed `gz: No such file or directory` and `RESULT: UNKNOWN`, which
      reads as a broken GPU rather than a missing PATH entry. Fixed.
- [x] **"Four FollowPath aborts in 143 s"** — withdrawn 2026-09-17. That number
      is `/navigator/status`'s `aborts=` field, which is
      `min(consecutive + 1, 4)` at `navigator.py:1595`, `:1644` and `:1684`,
      **reset to zero on success** at `:1675`, and incremented on goal
      *rejection* as well. It saturates at 4 and cannot distinguish 4 from 40.
      `autonomy_check.sh` samples it once at the end and never counted aborts.
      The true total is unknown; instrument it before tuning anything.
- [x] **"`lane_detection_node` cannot start without torch"** — withdrawn.
      `lane_detection.py` imports no torch; it is CLAHE, Canny and
      `HoughLinesP`. The node that needs torch is `lane_segmentation_node`.
- [x] **"The synchroniser never fires and logs nothing"** — withdrawn.
      `lane_detection.py:81-91` builds one synchroniser **per camera**, and
      `:119` plus `:924-929` install a 2 s watchdog that warns. Measured: with
      the front camera alone, the RGB and depth streams are aligned to a median
      stamp gap of **0.0000 s** and max 0.066 s, and the synchroniser as built
      at `:88-89` fired **396 times in 396 message pairs**.
- [x] **"The robot is 0.70 m wide"** — withdrawn. That is the Nav2 footprint
      parameter. The chassis collision mesh is **0.810 x 0.970 m**, and the
      chassis is the widest part, wider than the wheels.

---

## Not done

### P0 — blocking real perception work

- [x] **Hough lane detection already runs, in the Gazebo image, today.**
      *(2026-09-17)* `lane_detection.py` imports no torch, and the Gazebo image
      already carries `cv_bridge`, `message_filters`, `image_geometry`, numpy
      and OpenCV. Measured with the stock config and the robot driving itself
      for 90 s on the front camera alone: **`/lane_map` 253 occupied cells of
      102400, `/lane_costmap` 800 of 160000**, one startup watchdog warning,
      no errors. This is the cheap half of perception and it needed nothing.
- [ ] **Get YOLOPv2 running.** Only `lane_segmentation_node` needs torch, and
      the Gazebo image has none. `igvc-humble-fused-drive` has torch 2.14.0+cu130
      with `sm_120` in its arch list, and a real convolution runs on the
      Blackwell GPU (verified 2026-09-17), so the remaining question is whether
      a Humble container and a Jazzy container share a ROS graph. That is now
      the *only* thing the cross-container DDS gate has to prove. `models/`
      does not exist locally and `fetch_yolopv2_weights.sh` verifies no
      checksum.
- [x] **`num_cameras: 3` against one camera is a startup warning, not a
      failure.** *(2026-09-17)* Two separate claims were wrong: the node builds
      one synchroniser **per camera**, so camera 0 is unaffected, and it does
      log. Evidence above: 253 occupied cells with `sim_cameras:=front` and the
      stock `num_cameras: 3`. **No sim override is needed for function**; the
      target stays `sim_cameras:=all` for a three-camera budget measurement.
- [ ] **Verify the camera point cloud's orientation.** All four `rgbd_camera`
      outputs are tagged with the optical frame, which is right for the image
      and depth rasters and may rotate the cloud 90 degrees. Bridged but
      unused today. **Check before enabling `obstacle_layer`**, or Nav2 marks
      the floor lethal and the robot refuses to move for no visible reason.

### P1 — before competition

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
      it was wrong twice over. `GAZEBO_SETUP.md` 10.4 already retracts its own
      0.87 m gap, and the "0.70 m robot" is the Nav2 footprint, not the robot.
      Corrected: at the tightest barrel there is about 1.367 m of free cells,
      so a 0.810 m chassis **fits**, with roughly 0.557 m to spare. What does
      hold is that Nav2's `inflation_radius: 0.75` leaves **no zero-cost cell
      anywhere in that gap**, which is a cost-shape problem rather than a
      geometry one. Treat it as a hypothesis, not a diagnosis.
- [ ] **Run `render_check.sh` on the other machines.** The RTX 5080 Laptop and
      the Windows 10 machine are still untested, and the Intel-adapter crash
      depends on which GPUs a laptop has.
- [ ] **Verify `igvc_gazebo_linux` on a real Linux machine.** Written, never
      run. Delete the caveat in `DOCKER_CHANGES.md` 13.4 when it works.
- [ ] **TEAM: decide RQ-11, do barrels tip?** Gazebo has them solid and
      `static`; Isaac's generated field gives them **no collision at all**.
      The two simulators disagree right now. Real IGVC barrels tip.
- [ ] **Publish `/fix`.** The last unbridged row of the contract. Needs
      `<spherical_coordinates>` in the world first. Nothing consumes it today.

### P2 — worth doing

- [ ] **Settle the `/cmd_vel` timeout properly.** Asserted and withdrawn
      twice. Needs a test that timestamps the last command against the last
      odometry change rather than inferring a window from `ros2 topic echo`
      latency. `diff_drive_controller` does time out, so a difference runs in
      the unsafe direction.
- [ ] **Repair the four zero-normal collision meshes**, or decide the
      primitive colliders are permanent for simulation. They are still broken
      for Isaac and MoveIt.
- [ ] **Run `sim_cameras:=all` against the real course.** The three-camera
      budget was measured on a bare test scene, not this world.
- [ ] **Does YOLOPv2 tolerate 640x360?** The camera budget forces that
      resolution. If perception needs 720p, three cameras are not viable on
      this hardware - which is also the AGX-versus-Orin-Nano argument.
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

1. Fix `num_cameras` for the sim. One override.
2. Verify the point cloud orientation. One check.
3. Run perception out of `igvc-humble-fused-drive` against the Gazebo topics,
   rather than putting torch in the simulator image.
4. Compare its lane output against `/lane_ground_truth`, which is already
   published and is the exact right answer. **That comparison is free and
   nobody else gets it** - it is the strongest argument for having built the
   ground-truth path first.
