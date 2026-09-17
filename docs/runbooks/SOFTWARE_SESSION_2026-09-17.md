# Software session runbook, 2026-09-17, 6 to 8 pm

**Written to be read off a laptop while the room watches.** Liam runs this.

Success for tonight: **a teammate clones `main` on a machine nobody has
tested, follows the written guide, and ends with a robot driving the course.**

Everything here was verified today on Windows 11 Home build 26200, RTX 5070 Ti
Laptop, Docker Desktop 29.7.2, WSL2 Ubuntu-26.04. It has been run on **one
machine**. Say that out loud early; it sets expectations correctly and it is
true.

---

## 1. Pre-flight, before anyone arrives

Allow 20 minutes.

- [ ] **Push `main`.** The commands are at the end of the baseline report and
      in section 8 here. Nothing in this runbook works until `main` is on
      GitHub, because the whole session is people cloning it.
- [ ] **Confirm the push landed.** Open
      `https://github.com/wworth-IGVC/IGVC_robot_2027` and check the commit
      count and that `scripts/gazebo/bootstrap.sh` exists in the web view.
- [ ] **Put the image on the USB drive**, from a machine that has it:

      ```bash
      docker save igvc-gazebo-jazzy:latest -o igvc-gazebo-jazzy.tar
      ```

      1.20 GB, 14.6 s to write. Copy that one file to the drive. **Copy it to
      two drives if you have two**, because it is the single point of failure
      for the whole evening. Verify the copy is not truncated:

      ```bash
      tar -tf /mnt/d/igvc-gazebo-jazzy.tar | tail -4
      ```

      Want `index.json`, `manifest.json`, `oci-layout` and the `blobs/` lines.
- [ ] **Start Docker Desktop on your own machine and wait for `docker info` to
      answer.** It stops when the machine sleeps. If you carried the laptop
      here closed, it is not running.
- [ ] **Run the demo once, yourself, before anyone is watching.** If it fails
      in the room you lose twenty minutes; if it fails now you lose two.

      ```bash
      cd "/mnt/c/IGVC 2027/IGVC_robot_2027"
      docker compose -f docker-compose.windows.yml up -d igvc_gazebo
      docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
      ```
- [ ] **Then Ctrl-C it and leave the container up but idle.** Two simulators on
      one ROS domain is the worst failure you can have in front of people
      because it looks like a navigation bug. See section 3.
- [ ] **Check your own free disk**: `df -h /mnt/c`. Below 40 GB, stop and clear
      space before the meeting rather than during it.
- [ ] **Write the clone command on the whiteboard** before anyone opens a
      laptop. It is the one thing that must be typed exactly:

      ```text
      git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
      ```

---

## 2. The two hours, ordered so everyone reaches something

The ordering principle: **nobody waits on a download while you talk, and
nobody is stuck watching if their machine fails.** The long pole is per-machine
setup, so start it first and talk over it.

| Time | What | Why here |
| --- | --- | --- |
| **6:00** | USB drive goes round **immediately**. Everyone copies `igvc-gazebo-jazzy.tar` to their own disk, then passes it on. | It is one file and one drive. Start the serial resource first. |
| **6:05** | Everyone runs the clone command from the whiteboard, from a **WSL2 shell**. 20 seconds plus submodules. | Fast, and it surfaces who does not have WSL2 yet. |
| **6:10** | **The talk, while clones and copies finish.** Ten minutes, no slides. What the project is, why Gazebo and not Isaac, what works and what does not (section 6). | This is the only block that needs the room's attention, and it runs over the waiting. |
| **6:20** | Prerequisite check on every machine: `.\scripts\setup-windows.ps1` from PowerShell. Walk the room. | It names its own problems. This is where you find the Windows 10 machine and anyone without WSL Integration. |
| **6:30** | `IMAGE_TAR=/mnt/d/... bash scripts/gazebo/bootstrap.sh` on every machine. About 10 minutes: 77 s to load the image, then the build and the two checks. | Everyone is now doing the same thing at the same time and you can triage in parallel. |
| **6:45** | **Triage.** Section 3 is the three likeliest failures. Anyone whose machine is fine moves on to teleop; anyone stuck gets paired (section 4). | Deliberately generous. Machines are the risk, not concepts. |
| **7:05** | **Everyone drives the robot with the keyboard.** First thing anyone does that is not a check passing. | It is the moment people believe the simulator is real. Do this before the autonomy demo. |
| **7:20** | **The demo, in order, section 5.** Ends on the robot driving the course by itself. | The payoff, and by now everyone has their own copy so they can rerun it. |
| **7:40** | **The known-not-working list, read out loud**, section 6. Then the four questions for Aidan, section 7. | Honesty here is what stops someone reporting perception as working next week. |
| **7:55** | What each person will pick up. | |

**If the room runs behind, cut in this order:** the autonomy demo can be
watched on your screen instead of everyone's, and the known-not-working list
can be read while machines are still building. **Do not cut the prerequisite
check or teleop.** A teammate who leaves with a working simulator and no demo
is a success; one who watched a demo and has a broken clone is not.

---

## 3. The five failures you will actually see, one line each

**`RESULT: SOFTWARE FALLBACK (llvmpipe)`.** Gazebo is rendering on the CPU. Fix:
Docker Desktop, Settings, Resources, WSL Integration, enable their distro,
Apply and Restart; then check `docker exec igvc_gazebo ls /usr/lib/wsl/lib` is
not empty. **This is a warning, not a stop:** navigation, lidar, odometry and
control work are all still valid on llvmpipe, so send them to section 4 and
move on. Do not use `nvidia-smi` to diagnose it; it reports CUDA working fine
while OpenGL is on a software rasteriser.

**No window appears, and no error either.** They ran `docker compose` from
PowerShell. Fix: close it, open a WSL2 shell, `cd /mnt/c/...`, run it again.
Docker Desktop's Linux VM has no display, so a container started from
PowerShell can never open one and nothing errors. This is the single most
likely thing to happen tonight.

**Not enough disk.** Fix, in this order: `docker builder prune`, then
`docker image prune -a`. The offline path needs 12 GB free. Warn them that
this frees space *inside* the WSL2 VM but `docker_data.vhdx` on C: does not
shrink, and Windows 11 Home has no Hyper-V to compact it.

**`docker compose run` instead of `up -d` plus `exec`.** Fix: never use
`compose run`. It creates a **new** container with a random name every time,
so a second `run` gives a second, separate simulator rather than a second shell
into the first. Symptom to recognise: "it was working and now the robot drives
in circles at a barrel". That exact report was once six `gz sim` processes
running at once, and it was investigated as a navigation bug.

**Two simulators on one ROS domain.** Fix:
`docker compose -f docker-compose.windows.yml restart igvc_gazebo`, which is
the reliable reset. Both simulators publish `/clock`, `/odom` and `/tf` for the
same robot, so time and pose jump between them and Nav2 plans against a robot
that teleports. The results look real and are worthless.
`sim_preflight.sh` refuses to start a second one, and the scripts call it, but
be aware its own `pgrep -f` can self-match an inline `bash -c` whose text
contains the patterns, so confirm with `ps` if it looks wrong. **Also note
everyone is on `ROS_DOMAIN_ID=0`**: if two people on the same network both run
a simulator, they can see each other's. If the room's robots start behaving
strangely all at once, that is the cause, and the fix is a different
`ROS_DOMAIN_ID` per person.

**Bonus, and it will save you if it happens:** `colcon build` failing on a
package that looks unrelated, usually `zed_description`. Their clone has no
submodules. Fix: `git submodule update --init --recursive`.

---

## 4. A laptop with no working GPU path

Do not spend the meeting on it. Both options leave the person productive.

**Option A, headless plus RViz.** The simulator still runs; the cameras are
what software rendering cannot keep up with.

```bash
docker exec -it igvc_gazebo bash -c \
  "CAMERAS=none NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

RViz still shows the robot, the lidar returns and the odometry trail, so they
can watch the robot drive the course. They just cannot see the rendered world.
`gpu_lidar` falls back to software too and the physics is on the CPU
regardless, so **navigation, lidar, odometry and control work are all still
valid**. Tell them plainly: do not take camera or timing measurements and do
not compare them with anyone else's.

**Option B, pair.** Put them next to someone whose machine works. Two people
per machine is a better use of two hours than one person debugging Mesa. This
is the better option if the cause is a Windows 10 build below 19044, because
that machine **cannot** show a window until Windows is updated and there is no
workaround.

Either way, they should still finish `bootstrap.sh`, because everything except
the GPU check is machine-independent and they will want the clone working at
home.

---

## 5. What to demo, in order

Each step earns the next. Run them from a WSL2 shell, in the repo root.

**1. Prove the GPU, because everything visual depends on it.**

```bash
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
```

Point at the line `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)`
and say why it matters: the default failure is silent, 20x slower, and every
camera result from such a run is meaningless.

**2. Open the world and the robot.**

```bash
docker exec -it igvc_gazebo bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh
```

Two windows: Gazebo with the IGVC course, RViz with the robot, the `/scan`
returns and the camera. Worth saying: the course is **generated** from the same
`track_points.json` Isaac reads, so both simulators share one coordinate frame
with no alignment step. And the painted lane varies from 10 to 20 ft, straight
out of IGVC rules II.2, because a constant 12 ft lane was painted here until
today and the planner was planning in a different course from the one the
camera saw.

**3. Let someone else drive it.** Second WSL2 shell, same container:

```bash
docker exec -it igvc_gazebo bash
source /root/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` forward, `,` back, `j`/`l` turn, **`k` stop**. Warn them first: Gazebo's
DiffDrive holds the last command, so it keeps going when they let go of the
key. Measured today, it travelled a further 3.672 m in the 4 s after the
commands stopped. Hand the keyboard over; this is the moment it stops being a
demo and starts being theirs.

**4. The payoff: the robot drives the course by itself.** Ctrl-C the previous
one first.

```bash
docker exec -it igvc_gazebo bash -c "NAV=1 bash src/IGVC_robot_2026/scripts/gazebo/start_sim.sh"
```

About 60 seconds: Gazebo loads, Nav2 starts 15 seconds later on purpose, the
navigator plans, the robot sets off. While it drives, say what is in the loop:
the ground-truth lane grid, the IGVC navigator, Nav2's RegulatedPurePursuit
controller, the velocity smoother and the collision monitor. **All 2026 code,
unmodified, using the real robot's `nav2_lane_follow_config.yaml`.** Then
immediately say what is *not* in the loop, which is section 6.

**5. Optional, if there is time: the graded version.**

```bash
docker exec -it igvc_gazebo bash -c "bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
```

About five minutes and it **never publishes a velocity command**. Today's run:
83.0 m driven, max 1.86 m off the centreline, worst footprint clearance
+0.040 m, 0.0% of samples over the painted line. **If `IN LANE` says FAIL,
that is expected about one run in three and it is the test, not the robot.**
See section 6.

---

## 6. Known not working. Read this out loud.

One honest sentence each. This list is the reason nobody reports something
working next week that is not.

1. **Perception is not running across containers.** The lane lines and barrel
   positions come out of `track_points.json`, not out of the camera or the
   lidar; both publish and nothing plans on either, so what you just watched is
   a ground-truth navigation test and not obstacle detection.
2. **The cross-container link has an open defect.** Message counts between the
   Jazzy simulator and the Humble perception container match within 1% on every
   topic, but the Humble side prints `sequence size exceeds remaining buffer`
   three times per process even when it subscribes to nothing at all, so we do
   not yet trust YOLOPv2 results from it.
3. **The camera point cloud is rotated 90 degrees**, measured today and
   deliberately not fixed tonight, because fixing it means switching on a Nav2
   consumer that has never once been connected and that is a new system under
   test rather than a bug fix.
4. **`/fix` is not published**, so the GPS row of the interface contract is the
   one row still unbridged, and nothing consumes it today anyway.
5. **The FollowPath abort count everyone has been quoting is a saturating
   gauge**, `min(x+1, 4)` that resets on success and also counts goal
   rejections, so "four aborts" never meant four and nothing in this repository
   actually counts them.
6. **The `IN LANE` check is a coin flip** because it compares deviation against
   a flat 2.00 m tolerance while the lane width varies by a factor of two, and
   it needs a team decision rather than a bigger tolerance.
7. **Isaac Sim parity is deferred**, which is still the project's stated goal;
   the Gazebo side had to work first, and right now the two simulators disagree
   about whether barrels have collision at all.
8. **None of this has been run on any machine but one laptop**, so tonight is
   the first real test of the other three, and if it fails on yours that is
   information rather than your fault.

Two more worth saying if anyone asks about the real robot, because they affect
hardware and not just the sim: **`collision_monitor`'s point-cloud sources are
all disabled in the shipped config**, so its stop and slow polygons have no
live input on the real robot; and **`wheel_radius: 0.2032` in
`controllers.yaml` looks like the wheel diameter**, with the simulator
corroborating 0.1016 to about 0.1% on every run. Somebody should put a tape
measure on the wheel.

---

## 7. Four questions for Aidan

Chosen for what they unblock, not for what is most interesting.

1. **Where is the 2026 AGX now, and was its disk reflashed?** If the disk
   survives, two commands settle the entire deployment-topology question:
   `docker ps -a` looking for a container called **`dazzling_easley`**, and
   `systemctl status devmem-init`. That container name is hardcoded in all five
   of Nitin's competition scripts, and it was created by a bare `docker run`
   outside compose entirely, which is why arguing from which compose service is
   commented out has never settled it. **Unblocks:** whether the 2026 stack ran
   on the Jetson or a laptop, which decides what "parity" even means.
2. **You said the Dockerfile on `main` ran at competition. Could the image have
   been built from `main` while the code bind-mounted into it was
   `more_diverging_changes`?** `main` received exactly one unrelated commit
   during the competition window; `more_diverging_changes` received 54. That
   reconciles your answer with the history instead of contradicting it.
   **Unblocks:** which branch is the baseline we are trying to reach parity
   with. Adopting `more_diverging_changes` is a port and not a checkout: it
   deletes the entire simulator, restores the four meshes that segfault the
   physics engine, and rewrites every mesh path to `/home/nitin-5090/...`.
3. **Which lens is fitted to the three ZED X units, or does anyone still have a
   competition bag with a `camera_info` in it?** The simulator uses 100 degrees
   horizontal. The two candidate lenses give 104.63 and 74.06 degrees, so the
   sim is either 4.6 degrees too narrow or 26 degrees too wide and **we do not
   know the sign of the error.** One `camera_info` message settles it.
   **Unblocks:** every sim-versus-real camera claim, including anything that
   goes in the design report.
4. **RQ-06: `gz_ros2_control` in-process, or a `GazeboDriveHardware` bridged
   over topics?** Right now `ros2_control` is not in the loop at all; Nav2's
   twist goes straight to Gazebo's DiffDrive, so joint limits, the controller
   update rate and its interaction with the physics step are all untested.
   **Unblocks:** whether the simulator can ever test the control path, which is
   the difference between a navigation demo and a robot simulator. It has real
   control-loop-timing consequences and it is a team decision, not a launch
   file.

---

## 8. The push commands

Run these yourself. PowerShell, which is where you actually run them:

```powershell
cd "C:\IGVC 2027\IGVC_robot_2027"
git push origin main
```

Git Bash, if that is the shell in hand:

```bash
cd "/c/IGVC 2027/IGVC_robot_2027"
git push origin main
```

**Do not hand a `/c/...` path to PowerShell.** It resolves it as a relative
path, tries `C:\c\IGVC 2027\...`, fails, and the `git push` on the next line
then runs in whatever directory you were already in and reports "not a git
repository". PowerShell 5.1 also has no `&&`, which is why these stay two
lines.

`gazebo-phase1` is already merged into `main` as a fast-forward, so pushing
`main` is sufficient. If you also want the branch pointer on GitHub:

```powershell
git push origin gazebo-phase1
```

**Leave `docs-trim` alone.** It carries five unpushed commits of unknown
provenance that restructure `docs/`, and nobody on this side wrote them.
