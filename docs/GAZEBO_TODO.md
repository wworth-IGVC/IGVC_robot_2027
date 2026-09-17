# Gazebo simulator: what is done, and what is next

**Updated 2026-09-17.** A single page for "where is the simulator up to".
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

### Documentation

- [x] `GAZEBO_QUICKSTART.md` + `IGVC_2027_Gazebo_Setup_Guide.docx`, zero to a
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

### P0, blocking real perception work

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
- [ ] **Fix the point-cloud frame, and expect it to change behaviour.** Three
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
- [ ] **Close P0-1 properly: a partial pass with an open defect.** The
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
- [ ] **TEAM: decide the `IN LANE` gate.** `autonomy_check.sh` compares
      centreline deviation against a flat **2.00 m** tolerance while the lane
      width varies by a factor of two along the course, so it measures the
      wrong quantity and sits on its own threshold. At honest 15 Hz sampling
      it produced **2.04 (FAIL), 1.95 (PASS), 1.98 (PASS)**; one run in three
      fails. The behaviour did not regress: 47 to 51 m driven under its own
      control every time, on the slab throughout, **0.0% of samples over the
      paint** in all three. What changed is that the measurement got honest.
      **The regression baseline is unreliable until this is settled**, and it
      is a one-line change once decided. **Do not raise `TOL` to make it
      green.** The obvious candidate is to judge on the clearance metric,
      which is already computed and already lane-width aware, and to keep the
      centreline number as information.

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
- [ ] **TEAM: decide RQ-11, do barrels tip?** Gazebo has them solid and
      `static`; Isaac's generated field gives them **no collision at all**.
      The two simulators disagree right now. Real IGVC barrels tip.
- [ ] **Publish `/fix`.** The last unbridged row of the contract. Needs
      `<spherical_coordinates>` in the world first. Nothing consumes it today.

### P2, worth doing

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

Steps 1, 2 and 4 of the original four-step plan are done or dissolved. What is
left is narrower and harder than the original list implied.

1. ~~Fix `num_cameras` for the sim.~~ **Not needed.** One synchroniser per
   camera, so camera 0 is unaffected; 253 occupied cells with
   `sim_cameras:=front` against the stock `num_cameras: 3`.
2. ~~Verify the point cloud orientation.~~ **Done, and it is rotated 90
   degrees.** Fixing it is now its own item above, because it connects a Nav2
   consumer that has never been connected.
3. **Close the `sequence size exceeds remaining buffer` defect**, then run
   `lane_segmentation_node` out of `igvc-humble-fused-drive` against the
   Gazebo topics rather than putting torch in the simulator image. Needs
   `fetch_yolopv2_weights.sh` run, which verifies **no checksum**; note that
   when you do.
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
