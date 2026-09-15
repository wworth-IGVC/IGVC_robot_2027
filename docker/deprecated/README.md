# Deprecated images

**Nothing in this folder is the main use case.** It is kept for reference and
for reproducing old results, not for daily work.

The supported path is **ROS 2 Jazzy**. If you are looking for the simulator,
you want `docker/Dockerfile.gazebo-jazzy` and the `igvc_gazebo` service.

## Why Humble is being retired

Verified against the REP 2000 source on 2026-09-15:

| Distro | Released | End of life |
| --- | --- | --- |
| Humble Hawksbill | May 2022 | **May 2027** |
| Jazzy Jalisco | May 2024 | **May 2029** |

The competition is **4 to 8 June 2027**. Humble reaches end of life the month
before it, so the 2027 team would compete on an unsupported distro and the 2028
team would inherit one. Jazzy covers both.

Jazzy is also where the code already was, though not the robot images. The 2026
robot ran out of `DevEnv/jazzy_ws`, `ghcr.io/gold-rush-robotics/dev-zed` and
the org `dev_env` image are Jazzy, and the 2026 competition branch
`more_diverging_changes` is Jazzy code. The org robot-side images
(`jetson-zed`, `jetson-ros-base`, `jetson-isaac-ros`, `isaac-ros`) are still
Humble and are a separate migration - see section 5.4 of `DOCKER_CHANGES.md`,
and note that the "ROS 2 Jazzy" comments in `docker-compose.jetson.yml` are
stale.

## What is in here

| File | Replaced by | Status |
| --- | --- | --- |
| `Dockerfile.gazebo-harmonic` | `docker/Dockerfile.gazebo-jazzy` | Superseded. Same simulator, Gazebo Harmonic, on Humble instead of Jazzy. |

The Harmonic image was not wrong, and the findings it produced still hold. The
GPU render path, the four required WSL2 settings, the silent llvmpipe trap, the
no-display-from-PowerShell trap and the measured camera budget are all
properties of WSL2, Docker and the GPU, with nothing ROS-version-specific in
them. See `docs/GAZEBO_SETUP.md`, which documents them against the Jazzy image.

One thing genuinely changed for the better. On Humble, `gz_ros2_control` had no
Harmonic binary anywhere and was the single source build blocking Stage 2. On
Jazzy, `ros-jazzy-gz-ros2-control` ships as a binary against the same Gazebo.

## What is NOT in here yet

These are still Humble and are still the working images. They have **not** been
moved, because no verified Jazzy replacement exists yet and moving them would
leave the team with nothing for everyday work:

| File | Status |
| --- | --- |
| `docker/Dockerfile.humble-fused-drive` | Humble. Port to Jazzy not started. Still the everyday default. |
| `docker/Dockerfile.igvc-zed-humble` | Humble. Port to Jazzy not started. The private image it replaces, `dev-zed`, is itself Jazzy, so this port moves it closer to what it stands in for. |

Do not move them until their replacements build and pass the same checks.

## Building something in here anyway

The compose services still exist, suffixed so they cannot be reached by
accident:

```bash
docker compose -f docker-compose.windows.yml build igvc_gazebo_humble
```

Or directly, since this Dockerfile has no `COPY` and needs no build context:

```bash
docker build -t igvc-gazebo-harmonic:latest - < docker/deprecated/Dockerfile.gazebo-harmonic
```

## When this folder gets deleted

Once the fused-drive and ZED images are ported and verified on Jazzy, and
nobody has needed a Humble image for a while, delete this folder. Git keeps the
history; `git log --follow` works across the move.
