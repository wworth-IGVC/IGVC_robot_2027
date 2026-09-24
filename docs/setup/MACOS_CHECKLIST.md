# Mac verification checklist

**For whoever has the Mac. Please fill this in and hand it back.**

Nobody on this team has run the simulator on a Mac. The native Mac path in
[`MACOS.md`](MACOS.md) was verified on Linux with the same package recipes,
which proves a lot but cannot prove anything about Metal, the macOS Gazebo
window or macOS's own command-line tools. This file is how that changes.

**How to use it:** work down it in order, paste what actually happened into
each `ACTUAL` block, and return the file (a pull request, or send it to
whoever set you up). **A failure is as useful as a success.** If a step fails,
record it, note whether you could continue, and carry on.

About half an hour, most of it the first `pixi install`.

```text
Name:
Date:
Mac model (Apple menu, About This Mac):
Chip:
Memory:
macOS version:
```

---

## 1. The machine

```bash
uname -m
sw_vers -productVersion
sysctl -n hw.memsize | awk '{print $1/1073741824 " GB"}'
```

**Good:** `uname -m` prints **`arm64`**. If it prints `x86_64` your Terminal is
running under Rosetta: fix that first (`MACOS.md` section 2, step 3).

```text
ACTUAL
uname -m   =
macOS      =
memory     =
```

---

## 2. Tools

```bash
xcode-select -p          # command line tools present?
git --version
curl -fsSL https://pixi.sh/install.sh | bash
```

Open a **new** Terminal, then:

```bash
pixi --version
```

```text
ACTUAL
xcode-select -p =
git version     =
pixi version    =
```

---

## 3. Clone

```bash
git clone --recurse-submodules https://github.com/wworth-IGVC/IGVC_robot_2027.git
cd IGVC_robot_2027
git submodule status | wc -l          # want 9
git submodule status | grep '^-'      # want no output
```

```text
ACTUAL
submodule count  =
empty submodules =
```

---

## 4. Install the environment

```bash
time pixi install --locked
du -sh .pixi
```

**Good:** `The default environment has been installed.` On Linux this took
118 s and 5.8 GB; a Mac's download is similar in size. **If it fails, paste the
last 30 lines**; a solve failure here means `pixi.lock` and the channels
disagree about what exists for Apple Silicon.

```text
ACTUAL
succeeded (y/n) =
time            =
size of .pixi   =
errors, if any  =
```

---

## 5. Is it the environment we think it is

```bash
pixi run bash -c 'command -v ros2 gz; gz sim --versions; echo "runtime=$IGVC_RUNTIME ws=$IGVC_WS"; uname -m'
```

**Good:** both paths inside `.pixi/envs/default/bin`, `8.10.0`,
`runtime=pixi`, and `arm64`.

```text
ACTUAL

```

---

## 6. Build

```bash
time pixi run build
```

**Good:** `PASS  four packages built`. On Linux: 4 s.

```text
ACTUAL
result =
time   =
```

---

## 7. THE FIRST BIG ONE: does it render on the Mac's GPU

```bash
pixi run render-check
grep -iE "metal|apple|render ?system|gl_" ~/.gz/rendering/ogre2.log | head -20
```

**Good:** `camera topic : PUBLISHING`, `gpu_lidar topic: PUBLISHING`,
`RESULT: HARDWARE RENDERING (Apple GPU, Metal)` and **`YOUR TIER: M`**.

**This detection was written without a Mac to read the log from.** If you get
`YOUR TIER: unknown`, that is not your failure; paste the `grep` output above,
which is exactly what is needed to fix the detection. If either topic is
`SILENT`, the sensors are not rendering at all, which matters far more.

```text
ACTUAL
camera topic    =
gpu_lidar topic =
RESULT line     =
YOUR TIER       =
exit code       =
grep output:


```

---

## 8. The smoke test

```bash
time pixi run smoke
```

**Good:** `topic checks passed: 18   failed: 0`, `DRIVE TEST : PASS`,
`FRAME TEST : PASS`, heading disagreement near **0.13 deg** and distance ratio
near **0.997**. Those two numbers do not depend on rendering, so **a Mac should
reproduce them closely; if it does not, that is a real finding.**

The `timing :` line gives this machine's real time factor. Nothing in the test
depends on it, but it is the best single number for how fast a Mac is.

```text
ACTUAL
topic checks passed / failed =
DRIVE TEST / FRAME TEST      =
heading disagreement         =
distance ratio               =
timing line (real time factor) =
total time                   =
every FAIL line:


```

---

## 9. The autonomy check

```bash
time pixi run autonomy
```

**Good:** `AUTONOMY CHECK: PASS`. On Linux with software rendering: 89.2 m
driven over 100.0 s of simulation time, worst clearance +0.108 m. While it runs,
open Activity Monitor, Memory tab, and note the "Memory Used" figure.

```text
ACTUAL
AUTONOMY CHECK      =
distance travelled  =
IN LANE max         =
worst clearance     =
total time          =
Memory Used (Activity Monitor), roughly =
```

---

## 10. THE SECOND BIG ONE: the Gazebo window

```bash
pixi run sim
```

The launch file starts Gazebo's server and GUI as **two separate processes** on
macOS, because the `gz` command there refuses to run both in one. That branch
has **never run**. Expect a Gazebo window and an RViz window.

**Good:** a Gazebo window showing the course and the robot, and an RViz window
with the lidar and the camera image. Then, in a **second** Terminal:

```bash
cd IGVC_robot_2027
pixi shell
source .ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

and drive a little with `i`, `j`, `l`, `,`.

If the Gazebo window crashes or is blank, try the headless-plus-RViz route from
`MACOS.md` section 5 and record whether **that** works.

```text
ACTUAL
Gazebo window opened (y/n)       =
course and robot visible (y/n)   =
RViz opened, lidar and camera shown (y/n) =
robot drove with teleop (y/n)    =
did macOS ask about incoming network connections? what did you choose? =
errors, if any:


if the window failed: did headless:=true rviz:=true work? =
```

---

## 11. Optional: the Docker fallback

Only if you have Docker Desktop and time. Follow `MACOS.md` section 7 and
record just the outcome.

```text
ACTUAL
Virtual Machine Manager setting (Settings, General) =
bootstrap result     =
smoke test result    =
```

---

## Overall

```text
Did the native path reach 18 of 18?      =
Did render-check report tier M?          =
Did the Gazebo window work?              =
Would you use this Mac for simulator work? =
Anything that surprised you, or that MACOS.md got wrong:



```
