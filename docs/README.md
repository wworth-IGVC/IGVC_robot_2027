# docs/: what is in here and which file answers what

A map of this folder. Every file below is verified work, not notes: if something
here says a thing was measured, there is a command and a number behind it.

**Setting up a machine? Start at [`SETUP.md`](../SETUP.md) in the repo root.**
It picks your path by platform and sends you to one guide in `setup/`.

## `setup/`: one guide per platform

| File | For | One line |
| --- | --- | --- |
| `setup/WINDOWS.md` | Windows 10 / 11 | From a blank install to a robot driving the course, Docker Desktop + WSL2 |
| `setup/IGVC_2027_Windows_Setup_Guide.docx` | Windows | The same guide in Word, generated from `WINDOWS.md` |
| `setup/SESSION_COMMANDS.md` / `.docx` | Windows, in a meeting | Every command and link in session order, to copy and paste |
| `setup/MACOS.md` | Mac, Apple Silicon | The native path, pixi + RoboStack, plus the Docker fallback |
| `setup/MACOS_CHECKLIST.md` | Mac owners | The tear-off list that turns "untested on a Mac" into data. Please fill it in |
| `setup/LINUX.md` | Linux | pixi (recommended) or Docker on NVIDIA |
| `setup/make_setup_docx.py` | Maintainers | Builds a `.docx` from a guide in `setup/`; edit the Markdown, never the Word file |

## Everything else

| File | One line | Read it when |
| --- | --- | --- |
| `MAINTAINING_ENVIRONMENTS.md` | **How the Docker image and the pixi environment are updated, verified and published** | You own setup this year, or you are changing a dependency |
| `GAZEBO_TODO.md` | **What is done and what is next, one page** | Picking up simulator work, or reporting status |
| `GAZEBO_SETUP.md` | The simulator: which version, the GPU, the course, the robot, the interface contract, autonomy | Any Gazebo work |
| `GAZEBO_AGX_BASELINE_REPORT.md` | The running engineering report: every pass, its evidence and its corrections | You want the reasoning behind a decision |
| `DOCKER_CHANGES.md` | Every image and compose change, with evidence | You are changing an image, or wondering why one looks like that |
| `BRANCHES.md` | The 15 upstream branches, the submodule pin, what ran at competition | Choosing a baseline, or cloning |
| `RQ03_AUDIT.md` | Audit of what the downstream stack actually subscribes to | Wiring Gazebo to lane detection or Nav2 |
| `IGVC_2027_Docker_Setup_Guide.docx` | **Legacy.** The 2026-09-10 guide to the Humble perception images, not the simulator | Only for the Humble `igvc-humble-fused-drive` or ZED images |

---

## `IGVC_2027_Docker_Setup_Guide.docx`

**For:** a team member installing from scratch, on Windows, who has not used
Docker. It is deliberately the least technical document here and the only one in
Word rather than Markdown, because the team lead asked for something shareable.

**Contains:** Part 1 what Docker is and why we use it; Part 2 installing it,
including the WSL2 and WSLg prerequisites; Part 3 cloning the repository; Part 4
building and running each container, with a section on the Gazebo simulator;
Part 5 troubleshooting, including the four traps that have actually bitten
people; Part 6 known limitations; and a quick reference of every command.

**Careful:** Word locks the file. `make_docx.py` honours a `DOCX_OUT`
environment variable so it can be generated elsewhere and copied in.

## `docs/setup/IGVC_2027_Windows_Setup_Guide.docx` and `docs/setup/WINDOWS.md`

**For:** anyone who wants the simulator running and has not done it before. The
Markdown is the source of truth; the Word file is generated from it by
`docs/setup/make_setup_docx.py` and is overwritten on every run, so edit the Markdown.

```bash
python3 docs/setup/make_setup_docx.py
DOCX_OUT=/tmp/guide.docx python3 docs/setup/make_setup_docx.py   # if Word has it open
```

**Contains:** the WSL2-not-PowerShell rule and why it is absolute; the
prerequisites including the WSL Integration toggle people miss; build and run;
the GPU check that must pass before any camera work; running it with and
without autonomy; keyboard teleop; the two verification scripts; ten
troubleshooting entries taken from failures that actually happened; and a
section on what the simulator does NOT do, which is the part to read before
describing it to anyone.

## `GAZEBO_SETUP.md`

**For:** the simulator. This is the system of record for it; if Gazebo and this
document disagree, one of them is a bug.

**Contains,** in section order. **1** the short version, a table you can read in
thirty seconds. **2** why ROS 2 Jazzy and Gazebo Harmonic, with the end-of-life
arithmetic that decided it and the superseded Humble reasoning kept for
provenance. **3** the GPU, and why the default failure is a *silent* fall back
to CPU rendering that leaves every topic publishing at a plausible rate. **3A**
why you must launch from a WSL2 shell and never PowerShell, or there is no
window at all. **4** measured performance, including the camera budget that
forces 640x360. **4A** the verification log, with the actual command output.
**5** what is installed in the image and why. **6** the diagnostic scripts.
**7** what this still does not settle. **8** the generated IGVC course, the
bringup launch file, and the collision-mesh defect that used to crash the
physics engine. **9** RQ-03, the section 3.7 interface contract as one bridge
config, plus the two things that turned out not to be renames: Gazebo's
odometry frame is rotated by the spawn yaw, and a one-word frame-id typo named
a TF frame that did not exist. **10** the robot driving the course by itself -
what is in the loop, what it measured, and what it is emphatically not doing.

**The four things worth knowing before you touch anything:** rendering can fail
silently (section 3); `real_time_factor` lies about sensor rates because
physics and rendering run on separate threads, so you must count messages
(section 4); **the robot is not perceiving anything** - lane lines and barrels
come from `track_points.json`, not the camera (section 10.3); and two
simulators running at once produce results that look entirely plausible and are
worthless (section 10.5).

## `DOCKER_CHANGES.md`

**For:** the change record. It is long on purpose: it exists so nobody has to
re-derive a decision, and so a wrong turn is visible rather than quietly
deleted.

**Contains:** **1** summary. **2** the fixes that took the Humble image from
building 0 of 22 packages to 22 of 22. **3** compose changes across the three
files. **4** the consolidated ZED image that replaces a private one most team
accounts cannot pull, including the wrapper-and-SDK pairing the wrapper enforces
*at runtime*. **5** investigation findings, including the Humble/Jazzy audit of
the organisation's images. **6** supporting changes. **7** the verification log.
**8** the outstanding list, currently 18 items, which is the closest thing this
project has to a backlog. **9** the 2026 competition branch and what the images
were missing for it. **10** the Gazebo image and the two Windows traps it
exposed. **11** consolidating on ROS 2 Jazzy. **12** the world, the bringup
launch file, and a robot that drives.

**Start at section 8** if you are looking for something to do.

## `BRANCHES.md`

**For:** anything to do with which code is the baseline, and with cloning.

**Contains:** why there are 15 `2026/` mirror branches and how they were made; a
table of all 15 with their `isaac/exts` pin, commits ahead, and date; that pin
is the single most useful thing to know before checking one out; why the pin is
fragile, since ten branches reference a commit that exists only in the
organisation's fork and therefore breaks under `--depth 1`, breaks if that fork
is deleted, and does not survive mirroring off GitHub; the permanent fix, which
is about 25 lines in one file rather than repointing the submodule; what
actually ran at competition, traced through both compose files and verified
against the `docker_images` repository; and an organisation-wide branch audit
confirming no other repository hides competition work.

**The subsection people most need:** what a clone without `--recursive` gets
you. Nine submodules go missing, `colcon build` succeeds on exactly seven
packages, and it looks like a clean build while being an unusable robot: no
hardware interface, no perception, no lidar, no GPS.

## `RQ03_AUDIT.md`

**For:** connecting Gazebo to the rest of the stack. RQ-03 is the gap between "a
robot drives in simulation" and "lane detection and Nav2 run against it".

**Contains:** the output of a nine-agent audit: five parallel readers over
`igvc_lane_detection`, the Nav2 and navigator configuration, the real ZED
pipeline, the Isaac shim precedent and the `grr_hardware` control path, then a
design synthesis and three adversarial critics. It records a real frame-name bug
that predates RQ-03 and fails *silently*; four more failures that produce no
error at all; which renames a launch remapping cannot reach and why; and the
design conclusions that survived all three critics.

**It also contains its own biggest error, kept rather than deleted.** Its
headline claim, that the container could not run any downstream node, was
four-fifths wrong, and section 1 now explains both the correction and the method
mistake that produced it: a `grep` of a Dockerfile can only prove *not
explicitly listed*, which is not the same as *not present*. That is worth more
than the finding was.

**It is an audit, not an implementation.** Where it and the code disagree, the
code is newer.

---

## Where the rest of it lives

| Thing | Where |
| --- | --- |
| Diagnostic and bringup scripts | `../scripts/gazebo/`: start with `render_check.sh`, always |
| Image definitions | `../docker/`, retired ones in `../docker/deprecated/` with their own README |
| The generated course | `../src/igvc_test_description/worlds/` |
| How to run any of it | `../README.md` |

## Conventions these documents follow

- **A claim states its evidence.** A command, a file and line, or a measured
  number. "It works" on its own is not a finding here.
- **Superseded reasoning is struck through and kept, not deleted.** If a
  decision was reversed, both the original argument and the reason it fell are
  on the page, so the next person does not re-run the same reasoning and reach
  the same wrong answer.
- **Mistakes are recorded with the same weight as successes**, including the
  method error that produced them. Several in here were mine.
- **A test has to be able to fail.** A world with no robot still publishes
  `/clock`; a robot with the wrong wheel radius still publishes `/odom`. Checks
  in this project are written to fail on the thing they claim to check.


---

## Keeping these current

`docs/setup/WINDOWS.md` is the source; the `.docx` is generated from it and
overwritten on every run, so **edit the Markdown**:

```bash
python3 docs/setup/make_setup_docx.py
DOCX_OUT=/tmp/g.docx python3 docs/setup/make_setup_docx.py    # if Word has it open
```

Two things in the quickstart go stale fastest and are worth checking whenever
someone new sets up:

1. **Section 1A, the machine matrix.** The native-Linux service is written but
   unverified, and says so. When someone runs it successfully, delete that
   caveat in both `docs/setup/WINDOWS.md` and `DOCKER_CHANGES.md` 13.4.
2. **Section 9, troubleshooting.** Every entry there came from a failure that
   actually happened. When you lose an hour to something new, add it - that is
   what has made the list worth reading.
