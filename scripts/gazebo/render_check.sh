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
. "$(dirname "$0")/igvc_env.sh" || exit 1

WORLD="${1:-$(dirname "$0")/render_check.sdf}"
ITERATIONS="${ITERATIONS:-400}"
OGRE_LOG="$HOME/.gz/rendering/ogre2.log"
RUN_LOG="/tmp/render_check_gzsim.log"

# --headless-rendering selects EGL, which exists only on Linux. On macOS ogre2
# renders through Metal and the flag is left off. See gazebo_sim.launch.py.
HEADLESS_FLAG="--headless-rendering"
[ "$(uname -s)" = "Darwin" ] && HEADLESS_FLAG=""

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
timeout 120 gz sim -v 4 -s -r $HEADLESS_FLAG \
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

# ---------------------------------------------------------------------------
# Work out WHICH adapter is rendering, and therefore which support tier this
# machine is in. This used to grep only for nvidia|geforce|rtx, which meant a
# machine doing genuine hardware rendering on an Intel or AMD adapter was
# reported as UNKNOWN and read as broken. That is tier B, it is a supported
# configuration, and the script now says so. Fixed 2026-09-17.
# ---------------------------------------------------------------------------
GL_RENDERER_LINE=$(grep -m1 "GL_RENDERER" "$OGRE_LOG" 2>/dev/null)
ADAPTER=$(printf '%s' "$GL_RENDERER_LINE" | sed -e 's/.*GL_RENDERER[[:space:]]*=[[:space:]]*//' -e 's/[[:space:]]*$//')
[ -z "$ADAPTER" ] && ADAPTER="unknown"

SOFTWARE_HIT=$(grep -ciE "llvmpipe|softpipe|swrast|kms_swrast|software rasteri" "$OGRE_LOG")
NVIDIA_HIT=$(printf '%s' "$ADAPTER" | grep -ciE "nvidia|geforce|rtx|quadro|tesla")
INTEL_AMD_HIT=$(printf '%s' "$ADAPTER" | grep -ciE "intel|amd|radeon|arc\(tm\)|iris|uhd graphics|vega")

# macOS: ogre2 runs on Metal, so there is no GL_RENDERER line at all. The
# device name is taken from whatever line in the log names an Apple GPU. The
# exact log wording on a Mac is NOT VERIFIED as of 2026-09-24, so this falls
# through to tier "unknown" rather than guessing if it does not match.
METAL_HIT=0
if [ "$(uname -s)" = "Darwin" ]; then
    METAL_HIT=$(grep -ciE "metal" "$OGRE_LOG")
    if [ "$ADAPTER" = "unknown" ]; then
        APPLE_LINE=$(grep -m1 -iE "apple (m[0-9]|a[0-9]+)" "$OGRE_LOG")
        [ -n "$APPLE_LINE" ] && ADAPTER="Metal: $(printf '%s' "$APPLE_LINE" | sed 's/^[[:space:]]*//' | cut -c1-80)"
        [ -z "$APPLE_LINE" ] && [ "$METAL_HIT" -gt 0 ] && ADAPTER="Metal (device name not found in the log)"
    fi
fi

# TIER is written to a file so bootstrap.sh can read the verdict rather than
# re-deriving it, and cannot mistake a software machine for tier A.
TIER_FILE="${TIER_FILE:-/tmp/render_tier}"

echo "--- sensor data actually produced? ---"
# A render engine can initialise and still publish nothing. Check the topics.
timeout 20 gz sim -v 1 -s -r $HEADLESS_FLAG "$WORLD" > /dev/null 2>&1 &
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
echo " ADAPTER: $ADAPTER"
echo "=============================================================="
if [ "$SOFTWARE_HIT" -gt 0 ]; then
    echo " RESULT: SOFTWARE RENDERING (llvmpipe / swrast)"
    echo " YOUR TIER: C"
    echo
    echo " Nothing is wrong with your machine. The GPU is not reaching the"
    echo " container, so Mesa fell back to the CPU. This is a supported"
    echo " configuration and you can still do navigation, lidar, odometry"
    echo " and control work, because gpu_lidar falls back too and the"
    echo " physics runs on the CPU either way."
    echo
    echo " WHAT TO DO NEXT: follow the TIER C path in docs/setup/WINDOWS.md."
    echo " Run headless with RViz, keep the front camera only and keep it"
    echo " small. Do NOT take camera or timing measurements on this"
    echo " configuration and do not compare them with anyone else's."
    echo "=============================================================="
    echo "C" > "$TIER_FILE" 2>/dev/null
    exit 1
elif [ "$METAL_HIT" -gt 0 ] && [ -n "$CAM_MSG" ] && [ -n "$SCAN_MSG" ]; then
    echo " RESULT: HARDWARE RENDERING (Apple GPU, Metal)"
    echo " YOUR TIER: M"
    echo
    echo " This is the native macOS route (pixi / RoboStack) rendering on the"
    echo " Mac's own GPU, and both sensors produced data. Tier M is NEW and was"
    echo " not measured on any Mac before this script shipped, so please record"
    echo " this output in docs/setup/MACOS_CHECKLIST.md."
    echo "=============================================================="
    echo "M" > "$TIER_FILE" 2>/dev/null
    exit 0
elif [ "$NVIDIA_HIT" -gt 0 ]; then
    echo " RESULT: HARDWARE RENDERING (NVIDIA)"
    echo " YOUR TIER: A"
    echo
    echo " This is the measured baseline configuration. Everything in the"
    echo " quickstart applies to you as written."
    echo
    echo " WHAT TO DO NEXT: follow the TIER A path. Nothing special needed."
    echo "=============================================================="
    echo "A" > "$TIER_FILE" 2>/dev/null
    exit 0
elif [ "$INTEL_AMD_HIT" -gt 0 ]; then
    echo " RESULT: HARDWARE RENDERING (integrated Intel or AMD)"
    echo " YOUR TIER: B"
    echo
    echo " Your GPU IS reaching the container and rendering in hardware."
    echo " That is a real pass, not a partial one. Tier B is NOT measured"
    echo " by this project on any machine, so treat performance as unknown"
    echo " rather than assuming it matches tier A."
    echo
    echo " WHAT TO DO NEXT: follow the TIER A path, which should work. If"
    echo " gz sim crashes rather than running slowly, you are hitting the"
    echo " known Intel-adapter crash on machines that have BOTH an Intel"
    echo " iGPU and a discrete card: see MESA_D3D12_DEFAULT_ADAPTER_NAME in"
    echo " docker-compose.windows.yml, and docs/setup/WINDOWS.md tier B."
    echo " Please report your numbers back so this tier stops being"
    echo " unmeasured."
    echo "=============================================================="
    echo "B" > "$TIER_FILE" 2>/dev/null
    exit 0
else
    echo " RESULT: UNKNOWN"
    echo " YOUR TIER: unknown"
    echo
    echo " The render log named an adapter this script does not recognise:"
    echo "     $ADAPTER"
    echo " It is neither a known software rasteriser nor a vendor string"
    echo " this script knows. That most likely means a GPU vendor nobody on"
    echo " this team has tried, not a broken machine."
    echo
    echo " WHAT TO DO NEXT: inspect $OGRE_LOG by hand, and report the"
    echo " adapter string so it can be added here. Treat yourself as tier C"
    echo " until someone confirms otherwise."
    echo "=============================================================="
    echo "unknown" > "$TIER_FILE" 2>/dev/null
    exit 2
fi
