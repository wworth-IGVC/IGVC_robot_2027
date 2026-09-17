#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# RQ-02: prove Gazebo is rendering on the GPU, not silently on llvmpipe.
#
# Run this INSIDE the Gazebo container. It is the test that can actually fail:
# "gz sim launched" proves nothing, because a world with no rendering sensors
# starts fine on a machine that cannot render at all, and a world that does
# have them will still start, just on a software rasteriser at a few frames a
# second. Silent software fallback is the failure mode that invalidates every
# camera-based perception result, so it gets an explicit check.
#
# Exit status: 0 hardware rendering, 1 software fallback, 2 could not tell.
# ---------------------------------------------------------------------------

# Note: deliberately no `set -u`. Sourcing /opt/ros/$ROS_DISTRO/setup.bash reads
# AMENT_TRACE_SETUP_FILES unguarded and dies under nounset.
set -o pipefail

# `gz` is NOT on PATH in a fresh container. On this image Gazebo comes from the
# ROS vendor packages, so the binary lives at
# /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz and only appears on PATH once the
# ROS environment is sourced. /root/.bashrc does not source it, so the command
# this file's own header tells you to run
#     bash src/IGVC_robot_2026/scripts/gazebo/render_check.sh
# used to die with "gz: No such file or directory" and report RESULT: UNKNOWN,
# which reads as a broken GPU rather than a missing PATH entry. The other
# scripts here source it at the top; this one did not. Verified 2026-09-17.
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

WORLD="${1:-$(dirname "$0")/render_check.sdf}"
ITERATIONS="${ITERATIONS:-400}"
OGRE_LOG="$HOME/.gz/rendering/ogre2.log"
RUN_LOG="/tmp/render_check_gzsim.log"

echo "=============================================================="
echo " Gazebo render check"
echo "=============================================================="
echo "gz sim versions : $(gz sim --versions 2>/dev/null | tr '\n' ' ')"
echo "world           : $WORLD"
echo "iterations      : $ITERATIONS"
echo

if [ ! -f "$WORLD" ]; then
    echo "FAIL: world file not found: $WORLD"
    exit 2
fi

# Start from a clean log so we never read a previous run's verdict.
rm -f "$OGRE_LOG"

echo "--- running headless server with camera + gpu_lidar ---"
timeout 120 gz sim -v 4 -s -r --headless-rendering \
    --iterations "$ITERATIONS" "$WORLD" > "$RUN_LOG" 2>&1
GZ_STATUS=$?
echo "gz sim exit status: $GZ_STATUS"
echo

if [ ! -f "$OGRE_LOG" ]; then
    echo "RESULT: UNKNOWN - no ogre2 log at $OGRE_LOG"
    echo "        The render engine never initialised. Last 40 lines of run log:"
    tail -40 "$RUN_LOG"
    exit 2
fi

echo "--- GL / device lines from $OGRE_LOG ---"
grep -iE "vendor|renderer|version|gl_version|driver" "$OGRE_LOG" | head -20
echo

SOFTWARE_HIT=$(grep -ciE "llvmpipe|softpipe|swrast|kms_swrast|software rasteri" "$OGRE_LOG")
HARDWARE_HIT=$(grep -ciE "nvidia|geforce|rtx" "$OGRE_LOG")

echo "--- sensor data actually produced? ---"
# A render engine can initialise and still publish nothing. Check the topics.
timeout 20 gz sim -v 1 -s -r --headless-rendering "$WORLD" > /dev/null 2>&1 &
GZ_PID=$!
sleep 8
CAM_MSG=$(timeout 10 gz topic -e -n 1 -t /render_check/camera 2>/dev/null | head -5)
SCAN_MSG=$(timeout 10 gz topic -e -n 1 -t /render_check/scan 2>/dev/null | head -5)
kill "$GZ_PID" 2>/dev/null
wait "$GZ_PID" 2>/dev/null

if [ -n "$CAM_MSG" ]; then echo "camera topic   : PUBLISHING"; else echo "camera topic   : SILENT"; fi
if [ -n "$SCAN_MSG" ]; then echo "gpu_lidar topic: PUBLISHING"; else echo "gpu_lidar topic: SILENT"; fi
echo

echo "=============================================================="
if [ "$SOFTWARE_HIT" -gt 0 ]; then
    echo " RESULT: SOFTWARE RENDERING (llvmpipe / swrast)"
    echo " Three simulated cameras at 30 Hz will not run. Do not do"
    echo " camera-based perception work on this configuration."
    echo "=============================================================="
    exit 1
elif [ "$HARDWARE_HIT" -gt 0 ]; then
    echo " RESULT: HARDWARE RENDERING (NVIDIA)"
    echo "=============================================================="
    exit 0
else
    echo " RESULT: UNKNOWN - log named neither an NVIDIA device nor a"
    echo " software rasteriser. Inspect $OGRE_LOG by hand."
    echo "=============================================================="
    exit 2
fi
