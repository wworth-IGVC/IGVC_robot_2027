# Deprecated images

**Nothing in this folder is the main use case.** It is kept for reference and
for reproducing old results, not for daily work. No compose file has a service
for anything in here, on purpose.

The supported path is **ROS 2 Jazzy**:

| You want | Use |
| --- | --- |
| The simulator | `docker/Dockerfile.gazebo-jazzy`, service `igvc_gazebo` |
| Perception (torch, YOLOPv2), or the whole 22-package workspace | `docker/Dockerfile.dev-jazzy`, service `igvc_dev_jazzy` (`igvc_dev_jazzy_linux` on native Linux) |
| The 2026 competition code, with a ROS 2 terminal on it | `docs/COMPETITION_STACK.md` |

## Why Humble is being retired

Verified against the REP 2000 source on 2026-09-15:

| Distro | Released | End of life |
| --- | --- | --- |
| Humble Hawksbill | May 2022 | **May 2027** |
| Jazzy Jalisco | May 2024 | **May 2029** |

The competition is **4 to 8 June 2027**. Humble reaches end of life the month
before it, so the 2027 team would compete on an unsupported distro and the 2028
team would inherit one. Jazzy covers both.

Jazzy is also where the code already was. The 2026 competition branch
`more_diverging_changes` is Jazzy code, its scripts source `/opt/ros/jazzy`,
and the org `dev_env` image is Jazzy. What is still Humble is on the robot
side; see "Still Humble, and not in here" below.

## What is in here

| File | Replaced by | Since | Status |
| --- | --- | --- | --- |
| `Dockerfile.humble-fused-drive` | `docker/Dockerfile.dev-jazzy` (`igvc-dev-jazzy`) | 2026-09-29 | Retired from the compose files. Kept **for one season**, because the robot side has not been ported to Jazzy yet and someone may need to reproduce a Humble result. Delete it once nobody has needed it for a while |

The replacement was verified before this one was retired (report section 20):
the whole workspace builds 22 of 22 in it, every Python module imports, torch
runs a convolution on the GPU, YOLOPv2 runs against the simulator, both
simulator gates pass with it as the simulator container, and the cross-distro
DDS defect P0-1 is gone because nothing on the simulator's graph is Humble any
more.

**Deleted, 2026-09-29:** `Dockerfile.gazebo-harmonic`, the previous simulator
(Gazebo Harmonic on Humble), superseded by `docker/Dockerfile.gazebo-jazzy`
since 2026-09-15. Its findings (the GPU render path, the WSL2 settings, the
silent llvmpipe trap, the camera budget) were properties of WSL2, Docker and
the GPU and are documented against the Jazzy image in `docs/GAZEBO_SETUP.md`.
Git keeps it:

```bash
git show 88db437:docker/deprecated/Dockerfile.gazebo-harmonic
```

## Still Humble, and not in here

| File | Why it stays | Where its migration is planned |
| --- | --- | --- |
| `docker/Dockerfile.igvc-zed-humble` (services `igvc_zed_humble`, `igvc_zed_humble_upstream`) | **Robot side**: the ZED SDK and wrapper. Not part of the simulator path | `docs/JAZZY_MIGRATION.md` section 5: ZED SDK 5.5 with wrapper v5.5.0 on Jazzy, proven by launching the node |
| `jetson-zed:5.3-36.4.7` in `docker-compose.jetson.yml` | Robot side, an org image, JetPack 6 | the same section, and the JetPack decision |

## Building something in here anyway

It has no `COPY`, so it builds from the repo root in either shell:

```bash
docker build -f docker/deprecated/Dockerfile.humble-fused-drive -t igvc-humble-fused-drive:latest .
```

About 14 GB. Nothing else in the repo starts it any more.

## When this folder gets deleted

Once the robot-side images are ported and verified on Jazzy, and nobody has
needed a Humble image for a while, delete this folder. Git keeps the history;
`git log --follow` works across the move.
