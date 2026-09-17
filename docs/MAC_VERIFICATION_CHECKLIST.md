# Mac verification checklist

**For whoever has the MacBook. Please fill this in and hand it back.**

Nobody on this team has run the simulator on any Mac. Everything in
`MAC_SETUP.md` was written by reading the code and verified on Windows. This
file is the only way that changes.

**How to use it:** work down it in order, paste what actually happened into the
`ACTUAL:` line under each step, and return the file. **A failure is as useful
as a success, so do not stop and do not clean up the evidence.** If a step
fails, record it, note whether you could continue, and carry on to the next.

Fill these in first:

```text
Name:
Date:
Mac model (Apple menu, About This Mac):
```

---

## 1. The machine

**Command:**

```bash
uname -m
sw_vers
docker version
```

**What good looks like:** `uname -m` prints either `arm64` (Apple Silicon) or
`x86_64` (Intel Mac). **Which one decides whether everything below runs under
emulation**, so this is the most important single line in the file.

**ACTUAL:**

```text
uname -m        =
ProductVersion  =
docker version  =
```

---

## 2. Docker Desktop resources

**Where:** Docker Desktop, Settings, Resources.

**What good looks like:** memory 8 GB or more, 12 GB comfortable. Disk with at
least 20 GB free. If memory is below 8 GB, raise it before continuing and say
that you did.

**ACTUAL:**

```text
Memory allocated        =
CPUs allocated          =
Disk image size / free  =
Did you change anything =
Rosetta setting (Settings, General, "Use Rosetta for x86_64/amd64 emulation"):
   was it on or off     =
```

---

## 3. Clone

**Command:**

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
git submodule status | wc -l
git submodule status | grep '^-'
```

**What good looks like:** the count is **9** and the `grep` prints **nothing**.

**ACTUAL:**

```text
submodule count =
empty submodules =
time taken       =
```

---

## 4. Pull the image

**Command:**

```bash
time docker pull --platform linux/amd64 ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest
docker tag ghcr.io/wworth-igvc/igvc-gazebo-jazzy:latest igvc-gazebo-jazzy:latest
docker image ls igvc-gazebo-jazzy
```

**What good looks like:** the pull succeeds with no `no matching manifest`
error. About 1.2 GB over the wire. The image reports about 5.57 GB once the
container has run, and possibly ~1.2 GB before that, which is normal lazy
unpacking rather than a truncated pull.

**This is the first place Apple Silicon can go wrong.** If it refuses, paste
the exact error.

**ACTUAL:**

```text
succeeded (y/n) =
time taken      =
any warnings    =
image size      =
```

---

## 5. Does the container start

**Command:**

```bash
docker compose -f docker-compose.mac.yml up -d igvc_gazebo_mac
docker ps --filter name=igvc_gazebo_mac
docker exec igvc_gazebo_mac printenv ROS_DOMAIN_ID LIBGL_ALWAYS_SOFTWARE ROS_DISTRO
docker exec igvc_gazebo_mac uname -m
```

**What good looks like:** the container is `Up`. `LIBGL_ALWAYS_SOFTWARE` is
`1`, `ROS_DISTRO` is `jazzy`. **`uname -m` inside the container should say
`x86_64` even on an Apple Silicon Mac**, because the image is amd64 and is
being translated. That is the emulation working, not a mistake.

**ACTUAL:**

```text
container Up (y/n)      =
ROS_DOMAIN_ID           =
LIBGL_ALWAYS_SOFTWARE   =
uname -m inside         =
any warning printed by compose =
```

---

## 6. THE BIG ONE: does the workspace build under emulation

**This is the step most likely to fail, and the one nobody can predict.**
Compiling C++ under instruction translation is by far the heaviest thing in
the sequence. **Time it, even if it succeeds.**

**Command:**

```bash
time docker exec igvc_gazebo_mac bash -c '
  source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws &&
  colcon build --symlink-install \
    --base-paths /root/ros2_ws/src/IGVC_robot_2026/src \
    --packages-select zed_description igvc_test_description igvc_test_bringup igvc_lane_detection'
```

**What good looks like:** `4 packages finished`, exit 0. On the Windows
reference machine this takes **13 seconds** natively. **We have no idea what
it is under emulation.** If it takes twenty minutes, that is still a result;
write the number down.

**If it fails**, paste the first error, not the last. colcon aborts on the
first failure and one broken package masks the rest.

**ACTUAL:**

```text
succeeded (y/n)  =
TIME TAKEN       =
packages finished =
first error if it failed:


```

---

## 7. render_check.sh

**Command:**

```bash
time docker exec igvc_gazebo_mac bash -c "bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh"
echo "exit code: $?"
```

**What good looks like:**

```text
ADAPTER: llvmpipe (...)
RESULT: SOFTWARE RENDERING (llvmpipe / swrast)
YOUR TIER: C
```

**and exit code 1, which is correct.** Exit 1 means "software rendering", not
"broken". What would be a real failure is an OpenGL or render-engine **error**,
or `RESULT: UNKNOWN`.

**ACTUAL:**

```text
ADAPTER line     =
RESULT line      =
YOUR TIER        =
exit code        =
time taken       =
camera topic     =
gpu_lidar topic  =
```

---

## 8. bringup_smoke_test.sh, the one that matters

**Command:**

```bash
time docker exec igvc_gazebo_mac bash -c "bash src/IGVC_robot_2026/scripts/gazebo/bringup_smoke_test.sh"
echo "exit code: $?"
```

**What good looks like:** `topic checks passed: 18   failed: 0`, plus
`DRIVE TEST : PASS` and `FRAME TEST : PASS`. The distance ratio should be near
**0.998** and the heading disagreement near **0.13 deg**. Those two numbers
are physics and odometry, so they should be the same on a Mac as anywhere
else; **if they are not, that is a genuinely interesting finding.**

**If it is not 18 of 18, list every line that says FAIL.** Which topics fail
tells us whether the problem is the camera, the bridge or timing.

**ACTUAL:**

```text
topic checks passed / failed =
DRIVE TEST     =
FRAME TEST     =
distance ratio =
heading disagreement =
exit code      =
TIME TAKEN     =
every FAIL line:


```

---

## 9. autonomy_check.sh

**Command:**

```bash
time docker exec igvc_gazebo_mac bash -c "bash src/IGVC_robot_2026/scripts/gazebo/autonomy_check.sh"
echo "exit code: $?"
```

**What good looks like:** `AUTONOMY CHECK: PASS`. **`IN LANE` fails about one
run in three even on a good machine**, because that check is known to measure
the wrong thing; it is not a Mac problem. Everything else should pass.

A slow machine may genuinely change this result, because the robot has a fixed
wall-clock window to drive in. **If `MOVED` or `PROGRESS` fails, note the
distance travelled**: that is emulation being too slow rather than anything
being wrong.

**ACTUAL:**

```text
distance travelled  =
sample rate         =
MOVED / PROGRESS / IN LANE / ON THE SLAB =
AUTONOMY CHECK      =
exit code           =
TIME TAKEN          =
```

---

## 10. Did anything try to open a window

**Command, while a check is running, from a second Terminal:**

```bash
docker exec igvc_gazebo_mac bash -c 'ps -ef | grep -E "rviz2|gz sim gui" | grep -v grep'
```

**What good looks like: no output at all.** Nothing in the documented sequence
should start a GUI process. If something does, we have a bug in the launch
route and we need to know.

**ACTUAL:**

```text
any GUI process =
```

---

## 11. Clean up

```bash
docker compose -f docker-compose.mac.yml down
```

---

## 12. Overall

```text
Did you reach 18 of 18?           =
Total time from clone to smoke test =
Would you call this usable for
navigation work on this Mac?      =

Anything that surprised you, or that MAC_SETUP.md got wrong:



```

**Thank you. This is the only macOS data this project has.**
