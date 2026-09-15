#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# How fast, and for how long, does the render path actually survive?
#
# render_check.sh answers "GPU or llvmpipe". This answers the two questions
# that decide whether the configuration is usable for real work:
#
#   1. Achieved sensor rate. Acceptance criterion 6 needs the three-camera
#      scene at a usable rate on a 12 GB laptop; one camera well under its
#      requested rate means no.
#   2. Time to crash. The WSL d3d12 render path is fragile, so a run that
#      renders beautifully for four seconds and then dies is not a working
#      simulator, and the difference is invisible from a short test.
#
# Usage inside the container:
#   WORLD=/tmp/render_check.sdf \
#   BENCH_TOPICS="/render_check/camera /render_check/scan" \
#   bash sensor_bench.sh 60
#
# BENCH_TOPICS is a space-separated list; each is counted separately.
# ---------------------------------------------------------------------------
set -o pipefail

WORLD="${WORLD:-$(dirname "$0")/render_check.sdf}"
DURATION="${1:-60}"
BENCH_TOPICS="${BENCH_TOPICS:-/render_check/camera /render_check/scan}"
RUN_LOG=/tmp/sensor_bench_gzsim.log

# World name is needed for the stats topic; read it out of the SDF.
WORLD_NAME=$(grep -oE '<world name="[^"]+"' "$WORLD" | head -1 | sed 's/.*name="//; s/"//')

echo "=============================================================="
echo " Gazebo sensor benchmark: ${DURATION}s wall clock"
echo "=============================================================="
echo "world  : $WORLD  (world name: $WORLD_NAME)"
echo "topics : $BENCH_TOPICS"
echo

gz sim -v 2 -s -r --headless-rendering "$WORLD" > "$RUN_LOG" 2>&1 &
GZ_PID=$!

# Let the render engine come up before counting. Three 720p rgbd cameras take
# noticeably longer to initialise than one small camera.
sleep 15
if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "RESULT: server died during startup, before any measurement."
    tail -25 "$RUN_LOG"
    exit 1
fi
echo "server alive after 15s startup, measuring for ${DURATION}s..."
echo

START=$(date +%s)
IDX=0
COUNTER_PIDS=""
for T in $BENCH_TOPICS; do
    timeout "$DURATION" gz topic -e -t "$T" 2>/dev/null \
        | grep -c "^header" > "/tmp/bench_count_$IDX" &
    COUNTER_PIDS="$COUNTER_PIDS $!"
    IDX=$((IDX + 1))
done

DIED_AT=""
while [ $(( $(date +%s) - START )) -lt "$DURATION" ]; do
    if ! kill -0 "$GZ_PID" 2>/dev/null; then
        DIED_AT=$(( $(date +%s) - START ))
        break
    fi
    sleep 1
done

# Wait ONLY on the topic counters. A bare `wait` would also wait on the gz sim
# server, which runs forever, and the benchmark would never return.
for P in $COUNTER_PIDS; do
    wait "$P" 2>/dev/null
done

STATS=$(timeout 8 gz topic -e -n 1 -t "/world/$WORLD_NAME/stats" 2>/dev/null | tr '\n' ' ')

kill "$GZ_PID" 2>/dev/null
wait "$GZ_PID" 2>/dev/null

echo "=============================================================="
IDX=0
for T in $BENCH_TOPICS; do
    C=$(cat "/tmp/bench_count_$IDX" 2>/dev/null)
    [ -z "$C" ] && C=0
    printf " %-28s %6s msgs  ->  %s Hz\n" "$T" "$C" "$(( C / DURATION ))"
    IDX=$((IDX + 1))
done
echo " --------------------------------------------------------------"
if [ -n "$DIED_AT" ]; then
    echo " STABILITY : SERVER DIED after ${DIED_AT}s of measurement"
    echo
    echo " last lines of run log:"
    tail -15 "$RUN_LOG"
else
    echo " STABILITY : survived the full ${DURATION}s"
fi
echo
echo " sim stats : $STATS"
echo "=============================================================="

# real_time_factor is the number that decides usability.
RTF=$(echo "$STATS" | grep -oE 'real_time_factor: [0-9.]+' | head -1 | awk '{print $2}')
if [ -n "$RTF" ]; then
    echo " real_time_factor: $RTF"
fi
