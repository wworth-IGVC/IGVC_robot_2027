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

- [x] `render_check.sh` — GPU or silent software fallback.
- [x] `bringup_smoke_test.sh` — **18 of 18, 0 failures.** Contract topics carry
      messages, camera frame ids resolve through TF, and odometry is compared
      against Gazebo's own ground truth in heading *and* distance. That last
      check catches a wrong odom frame and a wrong wheel radius at once.
- [x] `autonomy_check.sh` — does the robot drive the course, never touching
      `/cmd_vel`.
- [x] `sim_preflight.sh` — refuses to start a second simulator on top of a
      first, because two on one ROS domain produce results that look real and
      are worthless.

### Documentation

- [x] `GAZEBO_QUICKSTART.md` + `IGVC_2027_Gazebo_Setup_Guide.docx` — zero to a
      driving robot, including a section for machines unlike the one this was
      built on.
- [x] `GAZEBO_SETUP.md` sections 9 and 10; `DOCKER_CHANGES.md` section 13.
- [x] `igvc_gazebo_linux` service so native Linux has something to run.

### Two measurements retracted, on purpose

- [x] **"No `/cmd_vel` timeout"** — withdrawn. The robot had driven off the
      ground slab and was falling with its wheels spinning, odometry counting
      up as though it were driving.
- [x] **"It circles at barrels"** — withdrawn. Six `gz sim` processes were
      running at once.

---

## Not done

### P0 — blocking real perception work

- [ ] **Get perception running at all.** The image has **no `torch`, no
      `ultralytics`, no YOLOPv2 weights**, so `lane_detection_node` cannot
      start. Decide where perception runs: add torch to the Gazebo image
      (roughly triples it), or run perception in `igvc-humble-fused-drive`
      against the Gazebo topics over DDS. **The second is probably right and
      nobody has tried it.**
- [ ] **`num_cameras` defaults to 3** in `lane_detection_config.yaml` while the
      sim runs one. The synchroniser then never fires *and logs nothing*.
      Needs a sim override before anyone debugs a "silent" lane detector.
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
- [ ] **Four FollowPath aborts in 143 s.** The navigator recovers by
      replanning, but `controller_server` logs `Failed to make progress` and
      the robot pauses. Tune before this is filmed for the design report. Start
      with the corridor arithmetic in `GAZEBO_SETUP.md` 10.4 - the gap past a
      barrel is 0.87 m for a 0.70 m robot, before Nav2's own 0.75 m inflation.
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
