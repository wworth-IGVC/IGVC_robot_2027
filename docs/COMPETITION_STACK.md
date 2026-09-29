# A ROS 2 terminal on the 2026 competition stack, from PowerShell

**What this does.** It runs the code that competed in 2026, branch
`2026/more_diverging_changes` at commit **`67a6934`**, in a container on your
laptop, started the way that branch's own `scripts/auton_launch.sh` starts
it. Then it opens a ROS 2 shell inside it, where `ros2 node list`,
`ros2 topic info`, `ros2 param get` and the rest work against the real
competition graph.

**Every command on this page is PowerShell.** No WSL2 shell, no Gazebo, no
window. Steps 1 to 6 are done once. Steps 7 to 9 are what you do each time.

**Verified 2026-09-29** on Windows 11 Home, RTX 5070 Ti Laptop 12 GB, Docker
Desktop 29.7.2, PowerShell 5.1. Every step was run from PowerShell on a fresh
container, twice. Measured: steps 1 to 6 take **13 s** when the image is
already built (the weights download took 6 s). Step 8 reported the stack
fully up after **7 to 14 s** of waiting. `check_stack.sh` found all **34 nodes** of the graph
measured on 2026-09-24, Nav2 active and YOLOPv2 loaded on `cuda:0`. One
thing could not be exercised: typing into an interactive `docker exec -it`
window. The same scripts were run without `-it`, and Ctrl-C was reproduced by
sending SIGINT to the launch's process group, which is what a console does.

---

## What is real here and what is not

| Part | In this container |
| --- | --- |
| The team's nodes: `lane_segmentation_node` (YOLOPv2), `igvc_navigator`, `mission_planner_node`, `lidar_obstacle_costmap_node`, `odom_tf_bridge`, `autonomous_indicator_node` | **Real**, from `67a6934`, unmodified |
| Nav2, 14 nodes, with the competition's `nav2_lane_follow_config.yaml` | **Real** |
| `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`, `twist_stamper` | **Real nodes.** The hardware under them is ros2_control's `mock_components/GenericSystem` instead of the ODrive over CAN. **This is the one change to the competition code**, made in the container's copy only, because there is no CAN bus |
| Three ZED X cameras, RPLidar C1, u-blox GPS | **Stand-ins** (`scripts/competition_stack/driver_standins.py`): the real node names and topic names, **no data**. The one exception: the front camera publishes a stationary odometry at 30 Hz, so TF exists and Nav2 can activate |
| A simulator | **None.** Nothing moves. Camera, lidar and GPS topics are silent |

**Good for:** learning and checking the competition graph. Which nodes run,
who publishes and subscribes to what, parameter values, lifecycle states,
controllers, TF. The node-by-node questions in the 2026-09-29 audit (four
nodes on `/cmd_vel_nav`, the smoother that cannot reverse, no `/odom`
publisher) can all be looked at here.

**Not for:** anything that needs sensor data or motion. For a robot that
drives, use the simulator, [`setup/WINDOWS.md`](setup/WINDOWS.md). Note that
the simulator runs this fork's `main` stack, not this competition branch.

---

## Before you start

1. **Docker Desktop is running.** It stops when the laptop sleeps. Start it
   from the Start menu and wait for the whale icon to stop animating.
2. **The repository is cloned with its submodules.** The commands assume
   `C:\IGVC 2027\IGVC_robot_2027`. If yours is elsewhere, change the `cd` in
   step 1; everything after it is relative.
3. **The image `igvc-dev-jazzy:latest` exists.** It is the Jazzy dev image:
   the Gazebo image plus torch (CUDA 13.0) and every dependency of the
   workspace. It is **not on GHCR**, so you build it once, from the repo
   root, in PowerShell:

   ```powershell
   docker compose -f docker-compose.windows.yml build igvc_dev_jazzy
   ```

   It needs `docker/Dockerfile.dev-jazzy`, which is on branch `jazzy-only`
   until that branch is merged. About 12 to 15 minutes on the laptop above
   (the torch download alone took 275 to 289 s on 2026-09-24), and
   **14.9 GB** on disk.
4. **An NVIDIA GPU.** The competition config pins YOLOPv2 to `cuda:0`
   (`lane_segmentation_config.yaml:175` at `67a6934`). Without one, expect
   `lane_segmentation_node` to fail to start while everything else runs.
   **Not tested.**
5. **Windows Terminal or the ordinary PowerShell window, not PowerShell ISE.**
   ISE cannot run interactive console programs, so `docker exec -it` does not
   work in it.

---

## Once: build the container (steps 1 to 6)

### 1. Go to the repo and check the three prerequisites

```powershell
cd "C:\IGVC 2027\IGVC_robot_2027"
docker info --format "{{.ServerVersion}}"
docker image inspect igvc-dev-jazzy:latest --format "{{.Id}}"
git cat-file -t 67a6934
```

Expect a version number, an image id, and the word `commit`:

```text
29.7.2
sha256:804d6f9578c7c158f958d5ac7706966d378fea03282d08df6685c4782f89fa7d
commit
```

Your image id will differ if you built the image yourself; that is fine. If
the last line is an error instead of `commit`, run `git fetch origin` and try
again.

### 2. Create the container

```powershell
docker run -d --name igvc_comp --gpus all --shm-size 2g -e ROS_DOMAIN_ID=73 -w /root/robot_ws igvc-dev-jazzy:latest sleep infinity
```

It prints a long container id. `sleep infinity` only keeps the container
alive; everything else runs in it through `docker exec`.

**`ROS_DOMAIN_ID=73` keeps it off everyone else's ROS graph**, including your
own simulator. Two stacks on one domain mix their nodes, and the result looks
plausible and is meaningless. Use a different number if 73 is already taken
on your machine.

### 3. Copy in the competition code, straight from git history

```powershell
git -c core.autocrlf=false archive --format=tar -o "$env:TEMP\igvc_comp_67a6934.tar" 67a6934 src/igvc_lane_detection src/igvc_test_bringup src/igvc_test_description
docker cp "$env:TEMP\igvc_comp_67a6934.tar" igvc_comp:/tmp/comp.tar
docker exec igvc_comp tar -xf /tmp/comp.tar -C /root/robot_ws
```

This reads commit `67a6934` out of git without checking it out, so **your
branch and working tree are not touched.** `-c core.autocrlf=false` stops
Git for Windows turning line endings into CRLF on the way.

### 4. Copy in `zed-description` and the helper scripts

```powershell
docker exec igvc_comp mkdir -p /root/comp /root/robot_ws/models
docker cp "src\zed-description" igvc_comp:/root/robot_ws/src/
docker cp "scripts\competition_stack\." igvc_comp:/root/comp/
```

`zed-description` comes from your checkout's submodule, version 0.1.5. The
competition branch pins 0.1.3; the difference is a material fix and two
camera models the robot does not carry. The graph measured on 2026-09-24 used
0.1.5 too.

### 5. Copy in the YOLOPv2 weights

If you have `models\yolopv2.pt` in the repo:

```powershell
docker cp "models\yolopv2.pt" igvc_comp:/root/robot_ws/models/
```

If you do not, download it straight into the container instead (156 MB):

```powershell
docker exec igvc_comp curl -fL -sS -o /root/robot_ws/models/yolopv2.pt https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt
```

Step 6 checks the file's SHA256 either way.

### 6. Build it

```powershell
docker exec igvc_comp bash /root/comp/setup_in_container.sh
```

Expect, in a few seconds:

```text
  PASS  competition packages, zed-description and weights are in place
  PASS  YOLOPv2 weights match the recorded SHA256
  PASS  ODrive CAN plugin swapped for mock_components/GenericSystem (container copy only)
        building 4 packages (a few seconds); log in /tmp/comp_build.log
  PASS  colcon build: Summary: 4 packages finished [2.89s]
  PASS  new shells in this container source ROS 2 Jazzy and the workspace

  Setup complete. Next: docs/COMPETITION_STACK.md step 7.
```

Any `FAIL` line names the step to go back to. Shown to fail on purpose: with
a weights file one byte longer, and with `igvc_test_bringup` missing, it
stops with exit 1. It is safe to run again.

---

## Every time: start it and open the terminal (steps 7 to 9)

### 7. Window 1: start the stack

```powershell
docker start igvc_comp
docker exec -it igvc_comp bash /root/comp/launch_stack.sh
```

`docker start` is harmless if the container is already running, and needed
after a reboot or a Docker Desktop restart. The second line starts the five
driver stand-ins, then `ros2 launch igvc_test_bringup
igvc_fused_drive.launch.py`, with `YOLOPV2_WEIGHTS` set exactly as
`auton_launch.sh` sets it. **Leave this window open**; the stack's output
scrolls here, and a copy goes to `/tmp/robot.log` in the container.

### 8. Window 2: check it is really up

Open a **second** PowerShell window:

```powershell
docker exec igvc_comp bash /root/comp/check_stack.sh --wait
```

`--wait` retries for up to 3 minutes. Expect:

```text
  PASS  all 32 named nodes of the 2026-09-24 graph are running
  PASS  no node is running twice
        nodes in the graph: 34 (34 on 2026-09-24)
  PASS  Nav2 lifecycle manager: Managed nodes are active
  PASS  lane_segmentation_node: YOLOPv2 ready: 1 instance(s) on cuda:0 (half=True).
  PASS  no process has died

  COMPETITION STACK: UP
  (ready after 7 s of waiting)
```

32 named nodes plus two tf2 helper nodes with random names make the 34.
Shown to fail on purpose: with nothing launched it fails every node check,
and with one node (`mission_planner_node`) killed it reports that node
missing, 33 nodes, exit 1.

### 9. The ROS 2 terminal

In window 2, or any new PowerShell window:

```powershell
docker exec -it igvc_comp bash
```

The prompt changes to `root@<container id>:/root/robot_ws#`. **ROS 2 Jazzy and
the workspace are already sourced**, and `ROS_DOMAIN_ID` is 73. Open as many
of these as you like. `exit` leaves the terminal; the stack keeps running.

---

## In the terminal: commands to try, with what they printed

Every output below was captured from this stack on 2026-09-29.

| Command | What it printed |
| --- | --- |
| `ros2 node list` | 34 nodes |
| `ros2 topic list` | 108 topics |
| `ros2 node info /igvc_navigator` | subscribes to the front ZED odom, `/lane_costmap`, `/lidar_obstacle_map`, `/mission/state`; publishes `/cmd_vel_nav`, `/lane_path`, `/navigator/status`; action clients `/follow_path`, `/navigate_to_pose` |
| `ros2 topic info -v /cmd_vel_nav` | `Publisher count: 7`: that is **4 nodes**, `mission_planner_node`, `igvc_navigator`, `controller_server` and `behavior_server`, the last holding four publishers. `Subscription count: 1`, `velocity_smoother`. Nothing arbitrates between them |
| `ros2 param get /velocity_smoother min_velocity` | `Double values are: [0.0, 0.0, -1.5]`: minimum forward speed 0, so the robot cannot reverse |
| `ros2 param get /controller_server FollowPath.desired_linear_vel` | `Double value is: 0.9` |
| `ros2 lifecycle get /controller_server` | `active [3]` |
| `ros2 control list_controllers` | `joint_state_broadcaster` and `diff_drive_controller`, both `active` |
| `ros2 topic hz /front_zed_camera_x/zed_node/odom` | `average rate: 30.0`, the stand-in's stationary odometry. Ctrl-C to stop |
| `ros2 run tf2_ros tf2_echo odom base_link` | two "Waiting for transform" lines for about 3 s, then an identity transform. Ctrl-C to stop |
| `ros2 topic echo --once /lidar_obstacle_map --field info` | a 1000 x 1000 grid at 0.1 m, origin (-50, -50) |
| `ros2 topic echo /scan` | **nothing**: the lidar is a stand-in with no data. Ctrl-C to stop |

---

## Stop, reset, remove

| To | Do | Measured |
| --- | --- | --- |
| Stop the stack | **Ctrl-C in window 1** | all 24 processes gone in about 1 s, nothing left behind. A line `[ERROR] [lane_segmentation_node-6]: process has died [... exit code -2 ...]` on the way out is that node taking the Ctrl-C itself, and is harmless |
| Start it again | step 7 | up in 14 s the second time |
| Reset anything stuck, or clear "ALREADY RUNNING" | `docker restart igvc_comp`, then step 7 | kills every process; the build and the sourced shell survive |
| After a reboot or a Docker Desktop restart | step 7 (its `docker start` brings the container back) | |
| Throw it all away | `docker rm -f igvc_comp`, then steps 2 to 6 | frees about 300 MB; the image stays |

`launch_stack.sh` refuses to start a second copy of the stack in the same
container, and tells you to `docker restart igvc_comp` instead: every node
twice would look healthy and be meaningless.

---

## PowerShell traps on this page

- **Do not pass bash commands with double quotes through `docker exec ...
  bash -c '...'`.** PowerShell 5.1 breaks double quotes inside arguments to
  programs like `docker`, and bash then reports `syntax error: unexpected end
  of file`. It happened while this page was being tested. Type bash commands
  in the ROS 2 terminal from step 9 instead. Every command on this page is
  written to avoid the problem.
- PowerShell 5.1 has no `&&`. Run commands one per line.
- `$env:TEMP` is PowerShell's name for your temporary folder; type it as
  written.

## When it goes wrong

| You see | Meaning | Fix |
| --- | --- | --- |
| `error during connect` or `failed to connect to the docker API` | Docker Desktop is not running, usually after sleep | Start it, wait, retry |
| `No such image: igvc-dev-jazzy:latest` | The image is not built | Before you start, item 3 |
| `fatal: Not a valid object name` in step 1 or 3 | Your clone does not have commit `67a6934` | `git fetch origin` |
| `Conflict. The container name "/igvc_comp" is already in use` in step 2 | It exists already | Skip to step 7, or `docker rm -f igvc_comp` and start again at step 2 |
| An error mentioning `gpu` or `capabilities` in step 2 | Docker sees no NVIDIA GPU. Not reproduced on this laptop, so the exact wording is not recorded here | Before you start, item 4. Removing `--gpus all` should let the rest run; untested |
| `the input device is not a TTY` | `-it` from somewhere without a console, such as PowerShell ISE | Use Windows Terminal or a plain PowerShell window |
| `ALREADY RUNNING` from step 7 | A stack, or its leftovers, is running | `docker restart igvc_comp`, then step 7 |
| `NOT READY` from step 8 after 3 minutes | Something failed at startup | `docker exec igvc_comp tail -n 60 /tmp/robot.log` |
| `ros2: command not found` in the terminal | The shell was opened before step 6, or with `sh` instead of `bash` | `source /root/robot_ws/install/setup.bash` |

---

## How close this is to the robot, and to the 2026-09-24 measurement

**Against the robot at the competition.** The branch's scripts start the
sensors with `sensor_launch.sh` and the autonomy with `auton_launch.sh`, both
in a container named `dazzling_easley`, with the workspace built at
`/root/ros2_ws/src/IGVC_robot_2026`. Here the sensors are stand-ins, the
ODrive is a mock, the container is `igvc_comp` and the workspace is
`/root/robot_ws`. The top-level launch, its arguments, the configs and the
nodes are the branch's. **That the robot ran exactly this branch at the
competition is not confirmed**: the team lead has said the Dockerfile on
`main` competed, and whether the code mounted into it was this branch is an
open question to him.

**Against the graph measured on 2026-09-24** (off the repo, in
`local-notes/ros2_graph_robot_2026-09-24` on Liam's machine, and the ROS
Graph Explorer built from it; ask Liam for the link). Same image, commit, packages, stand-ins,
plugin swap and launch. Three differences, none of which changed the result
(34 nodes both times): `--shm-size 2g` (the compose files' value; the
measurement used Docker's 64 MB default), `-w /root/robot_ws`, and the
archive made with `core.autocrlf=false`. Some files in the competition commit
are stored with CRLF line endings in git itself; they are copied as they
are, exactly as the robot had them.

## Files

| File | What |
| --- | --- |
| `scripts/competition_stack/setup_in_container.sh` | Step 6: checks inputs and the weights' SHA256, swaps the ODrive plugin for the mock, builds the 4 packages, sources new shells |
| `scripts/competition_stack/launch_stack.sh` | Step 7: stand-ins plus `igvc_fused_drive.launch.py`; refuses to start twice |
| `scripts/competition_stack/check_stack.sh` | Step 8: compares the live graph with the 2026-09-24 one; `--wait` retries |
| `scripts/competition_stack/driver_standins.py` | The five driver stand-ins, unchanged from the 2026-09-24 measurement |

For the branch itself, what it adds to `main` and which parts ran, see
[`BRANCHES.md`](BRANCHES.md) and
[`JAZZY_MIGRATION.md`](JAZZY_MIGRATION.md).
