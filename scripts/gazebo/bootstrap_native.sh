#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# One command from a fresh clone to a verified simulator, WITHOUT Docker.
#
# THIS IS THE macOS PATH (Apple Silicon). It also works on native Linux. It
# runs the simulator from the pixi environment defined in pixi.toml, which
# installs ROS 2 Jazzy, Gazebo Harmonic and Nav2 from RoboStack. Windows users
# want scripts/gazebo/bootstrap.sh instead; see SETUP.md at the repo root.
#
# Do not run this file directly. From the repository root:
#
#   pixi run bootstrap                # everything below
#   pixi run build                    # steps 1 to 3 only
#
# What it does, printing PASS or FAIL for each step and stopping at the first
# failure:
#
#   1. sanity: right repo, inside the pixi environment, supported machine, disk
#   2. git submodule update --init --recursive
#   3. colcon build the four packages the simulator needs
#   4. render_check.sh        is the GPU rendering, and which tier is this
#   5. bringup_smoke_test.sh  does the topic contract hold, does it drive
#
# Options, as environment variables:
#   SKIP_SMOKE=1    stop after render_check.sh
#
# Same rules as the Docker bootstrap: every step prints a line, a step that
# cannot run says so, and it never reports the GPU working when it is not.
# ---------------------------------------------------------------------------
set -o pipefail

STEP=0
FAILED=""
say()  { printf '\n%s\n' "$*"; }
pass() { printf '  PASS  %s\n' "$*"; }
info() { printf '        %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*"; }
fail() {
    printf '\n  FAIL  %s\n' "$*"
    printf '\n  Stopped at step %s. Nothing after this point has run.\n' "$STEP"
    if [ -n "$FAILED" ]; then printf '  %s\n' "$FAILED"; fi
    printf '\n  docs/setup/MACOS.md section 5 lists the usual causes.\n\n'
    exit 1
}
step() { STEP=$((STEP + 1)); printf '\n==> step %s: %s\n' "$STEP" "$*"; }

BUILD_ONLY=0
[ "${1:-}" = "--build-only" ] && BUILD_ONLY=1

printf '  ---------------------------------------------------------------\n'
printf '  IGVC 2027 Gazebo bootstrap, native (pixi / RoboStack)\n'
printf '  ---------------------------------------------------------------\n'

# ---------------------------------------------------------------------------
step "checking where we are and what we have"
# ---------------------------------------------------------------------------
if [ ! -f pixi.toml ] || [ ! -d scripts/gazebo ]; then
    FAILED="Run it from the repository root with: pixi run bootstrap"
    fail "this is not the repository root. cwd is $(pwd)"
fi
pass "in the repository root"

if [ "${IGVC_RUNTIME:-}" != "pixi" ] || ! command -v ros2 >/dev/null 2>&1; then
    FAILED="Use 'pixi run bootstrap' from the repo root, not 'bash scripts/...'."
    fail "not running inside the pixi environment, so ROS 2 is not on PATH"
fi
pass "inside the pixi environment  (ROS_DISTRO=${ROS_DISTRO:-unset})"

OS=$(uname -s); ARCH=$(uname -m)
case "$OS/$ARCH" in
    Darwin/arm64)
        pass "macOS on Apple Silicon ($(sw_vers -productVersion 2>/dev/null || echo '?'))" ;;
    Darwin/x86_64)
        # Either an Intel Mac, which this project does not support, or a Terminal
        # running under Rosetta, which gets x86 packages and no Metal speed-up.
        FAILED="On Apple Silicon, open Terminal without 'Open using Rosetta'. Intel Macs are not supported."
        fail "this shell is x86_64 on macOS" ;;
    Linux/x86_64)
        pass "Linux x86_64 (native pixi route)" ;;
    *)
        warn "$OS/$ARCH is not a platform pixi.toml lists; expect the install to have failed" ;;
esac

MEM_GB=""
if [ "$OS" = "Darwin" ]; then
    MEM_GB=$(( $(sysctl -n hw.memsize 2>/dev/null || echo 0) / 1073741824 ))
else
    MEM_GB=$(awk '/MemTotal/ {print int($2/1048576)}' /proc/meminfo 2>/dev/null)
fi
if [ -n "$MEM_GB" ] && [ "$MEM_GB" -gt 0 ]; then
    if [ "$MEM_GB" -lt 12 ]; then
        warn "${MEM_GB} GB of RAM. The full stack (Gazebo, Nav2, lane detection)"
        info "was measured using several GB; see docs/setup/MACOS.md section 1."
        info "Close other apps, and prefer the headless checks."
    else
        pass "${MEM_GB} GB of RAM"
    fi
fi

NEED_GB=10
FREE_GB=$(df -k . 2>/dev/null | tail -1 | awk '{print int($4/1048576)}')
if [ -z "$FREE_GB" ]; then
    warn "could not read free disk. WANTS ${NEED_GB} GB. Continuing anyway."
elif [ "$FREE_GB" -lt "$NEED_GB" ]; then
    fail "not enough free disk.  WANTS ${NEED_GB} GB   FOUND ${FREE_GB} GB"
else
    pass "free disk:  WANTS ${NEED_GB} GB   FOUND ${FREE_GB} GB"
fi

# ---------------------------------------------------------------------------
step "submodules"
# ---------------------------------------------------------------------------
if ! git submodule update --init --recursive 2>&1 | tail -20; then
    FAILED="Check network access to github.com, then run: git submodule update --init --recursive"
    fail "git submodule update failed"
fi
EMPTY=$(git submodule status | grep -c '^-')
if [ "$EMPTY" -ne 0 ]; then
    FAILED="Run: git submodule update --init --recursive"
    fail "$EMPTY submodule(s) are still empty. The build will fail without them"
fi
pass "all $(git submodule status | wc -l | tr -d ' ') submodules populated"
for req in IGVC_track_generator/track_points.json src/zed-description/package.xml; do
    [ -f "$req" ] || fail "$req is missing, and the simulator needs it"
done
pass "track data and zed_description are on disk"

# ---------------------------------------------------------------------------
step "building the workspace"
# ---------------------------------------------------------------------------
# shellcheck disable=SC1091
. scripts/gazebo/igvc_env.sh || fail "scripts/gazebo/igvc_env.sh could not be sourced"
T0=$(date +%s)
if ! igvc_build; then
    FAILED="The log above is /tmp/colcon.log. A missing compiler usually means 'pixi install' did not finish."
    fail "colcon build failed"
fi
pass "four packages built in $(( $(date +%s) - T0 )) s, workspace at $WS"

if [ "$BUILD_ONLY" = "1" ]; then
    say "  Build complete. Next: pixi run bootstrap (or pixi run smoke)."
    exit 0
fi

# ---------------------------------------------------------------------------
step "render_check.sh: is the GPU rendering"
# ---------------------------------------------------------------------------
RENDER_OUT=$(bash scripts/gazebo/render_check.sh 2>&1)
RENDER_RC=$?
printf '%s\n' "$RENDER_OUT" | grep -E "ADAPTER:|RESULT:|YOUR TIER|topic" | sed 's/^/        /'
TIER=$(printf '%s' "$RENDER_OUT" | sed -n 's/.*YOUR TIER:[[:space:]]*\([A-Za-z]*\).*/\1/p' | head -1)
case "$TIER" in
    M) pass "TIER M: the Mac's GPU is rendering through Metal" ;;
    A|B) pass "TIER $TIER: hardware rendering" ;;
    C) warn "TIER C: software rendering. The smoke test still runs; cameras will be slow." ;;
    *) warn "TIER ${TIER:-unreported} (render_check exit $RENDER_RC). Record the output above"
       info "in docs/setup/MACOS_CHECKLIST.md step 7; the smoke test runs anyway." ;;
esac

if [ "${SKIP_SMOKE:-0}" = "1" ]; then
    warn "SKIP_SMOKE=1: the simulator is NOT verified. Re-run without it."
    exit 0
fi

# ---------------------------------------------------------------------------
step "bringup_smoke_test.sh: the topic contract, and does it drive"
# ---------------------------------------------------------------------------
bash scripts/gazebo/bringup_smoke_test.sh > /tmp/bootstrap_smoke.log 2>&1
SMOKE_RC=$?
grep -E "topic checks passed|DRIVE TEST|FRAME TEST|heading disagreement|distance ratio|READY" \
    /tmp/bootstrap_smoke.log | sed 's/^/        /'
if [ "$SMOKE_RC" -ne 0 ]; then
    FAILED="Full log: /tmp/bootstrap_smoke.log. Every FAIL line in it is worth reporting."
    fail "bringup_smoke_test.sh exited $SMOKE_RC"
fi
pass "smoke test passed"

say "  Bootstrap complete. The simulator is verified on this machine."
say "  Next: pixi run sim        (the simulator, with a Gazebo window)"
say "        pixi run autonomy   (the robot drives the course by itself)"
