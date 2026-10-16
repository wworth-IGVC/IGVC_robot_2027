#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# F17 check D: how often does Nav2's lifecycle bring-up abort, with Gazebo
# running, measured one bring-up at a time instead of one 5-minute drive at a
# time?
#
#   bash scripts/gazebo/measurements/nav2_bringup_loop.sh N [LABEL]
#
# Run INSIDE the Gazebo container. Starts the simulator ONCE (gazebo_nav_test
# with nav2:=false: Gazebo, bridge, cloud re-stamp, ground-truth grids, the
# same /scan and point-cloud sources collision_monitor subscribes to), then N
# times: start Nav2 alone with the sim's own params, wait for "Managed nodes
# are active" or "Aborting bringup", record it, stop Nav2, repeat. About 25 to
# 40 s per iteration, so N=100 is about an hour, against 5 to 9 minutes per
# drive in autonomy_check.sh.
#
# WHY IT EXISTS. Two aborts in 59 saved bring-ups (3.4 %, 95 % interval 0.4 to
# 12 %) is too rare to measure with full drives: seeing one abort with 95 %
# confidence takes about 80 drives. A bring-up alone costs a tenth of that.
#
#   env  LOAD=none|spin   spin = busy loops on LOAD_CPUS cores (default half of
#                         nproc) for the whole loop: deliberate extra load
#        VARIANT=normal|no_cloud   no_cloud = collision_monitor WITHOUT its
#                         point-cloud source. A DIAGNOSIS arm only: it changes
#                         what the simulator exercises (the real robot's monitor
#                         has the cloud). If the abort rate falls with the cloud
#                         gone, the cloud stream is implicated; keep it out of
#                         any gate.
#        PER_RUN_LIMIT=90 seconds to wait for a verdict before calling it a
#                         TIMEOUT (counted, never an abort and never a pass)
#        OUT=/tmp/f17_<label>   where logs and the CSV go
#
# Per iteration it records the UDP RcvbufErrors delta (container network
# namespace; a kernel-side drop of datagrams for a socket that read too slowly),
# the 1-minute load average and the CPU pressure stall figure if the kernel
# has one, so an abort can be set beside the machine's state. On an abort it
# also saves `ss -uamn` (per-socket drop counters) and the busiest threads.
#
# NOT RUN when written (no ROS on the authoring machine): bash -n only.
# ---------------------------------------------------------------------------
set -o pipefail
set -m     # job control, so each background job is its own process group

HERE="$(cd "$(dirname "$0")" && pwd)"
. "$HERE/../igvc_env.sh" || exit 1
. "$HERE/../sim_preflight.sh"

N="${1:?usage: nav2_bringup_loop.sh N [LABEL]}"
LABEL="${2:-run}"
LOAD="${LOAD:-none}"
VARIANT="${VARIANT:-normal}"
PER_RUN_LIMIT="${PER_RUN_LIMIT:-90}"
OUT="${OUT:-/tmp/f17_${LABEL}}"
mkdir -p "$OUT"
CSV="$OUT/loop.csv"
echo "i,result,secs_to_verdict,rcvbuf_errors_delta,loadavg1,cpu_psi_some_avg10" > "$CSV"

PARAMS="$REPO/src/igvc_test_bringup/config/nav2_lane_follow_config.yaml"
EXTRA="$REPO/src/igvc_test_bringup/config/gazebo_nav_test_nav2_overrides.yaml"
if [ "$VARIANT" = "no_cloud" ]; then
    EXTRA="$OUT/overrides_no_cloud.yaml"
    python3 - "$REPO/src/igvc_test_bringup/config/gazebo_nav_test_nav2_overrides.yaml" "$EXTRA" <<'PY' || exit 1
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
d["collision_monitor"]["ros__parameters"]["observation_sources"] = ["lidar_scan"]
yaml.safe_dump(d, open(sys.argv[2], "w"))
PY
fi

udp_rcvbuf () { awk '/^Udp:/ {n++; if (n == 2) print $6}' /proc/net/snmp; }
psi_cpu ()    { sed -n 's/^some .*avg10=\([0-9.]*\).*/\1/p' /proc/pressure/cpu 2>/dev/null | head -1; }
nav2_procs () { pgrep -f "${IGVC_ROS_PREFIX:-/opt/ros/jazzy}/lib/nav2_" 2>/dev/null; }

SPIN=()
cleanup () {
    for p in "${SPIN[@]}"; do kill "$p" 2>/dev/null; done
    [ -n "${NAV:-}" ] && kill -INT -- "-$NAV" 2>/dev/null
    [ -n "${SIM:-}" ] && kill -INT -- "-$SIM" 2>/dev/null
    sleep 3
    sim_teardown >/dev/null 2>&1
}
trap cleanup EXIT

echo "=== F17 nav2 bring-up loop: N=$N label=$LABEL load=$LOAD variant=$VARIANT out=$OUT ==="
FORCE=1 sim_preflight || exit 1
igvc_build || exit 1

ros2 launch igvc_test_bringup gazebo_nav_test.launch.py headless:=true nav2:=false \
    > "$OUT/sim.log" 2>&1 &
SIM=$!
if ! python3 "$HERE/../wait_ready.py" odom 300; then
    echo "FAIL: the simulator never became ready"; exit 1
fi
# The real run starts Nav2 15 s after Gazebo; give the clouds that long to be flowing.
sleep 15

if [ "$LOAD" = "spin" ]; then
    K="${LOAD_CPUS:-$(( $(nproc) / 2 ))}"
    echo "load: $K busy loops"
    for _ in $(seq 1 "$K"); do ( while :; do :; done ) & SPIN+=("$!"); done
fi

ok=0; abort=0; timeout_n=0
for i in $(seq 1 "$N"); do
    log="$OUT/nav2_$i.log"
    r0=$(udp_rcvbuf); t0=$(date +%s.%N)
    ros2 launch igvc_test_bringup navigation_no_docking.launch.py \
        params_file:="$PARAMS" extra_params_file:="$EXTRA" use_sim_time:=true \
        > "$log" 2>&1 &
    NAV=$!
    res=TIMEOUT
    while :; do
        if grep -q "Managed nodes are active" "$log" 2>/dev/null; then res=OK; break; fi
        if grep -q "Aborting bringup" "$log" 2>/dev/null; then res=ABORT; break; fi
        now=$(date +%s.%N)
        if awk -v a="$t0" -v b="$now" -v c="$PER_RUN_LIMIT" 'BEGIN{exit !(b-a>c)}'; then break; fi
        sleep 0.5
    done
    t1=$(date +%s.%N)
    secs=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.1f", b-a}')
    r1=$(udp_rcvbuf)
    load1=$(cut -d' ' -f1 /proc/loadavg)
    psi=$(psi_cpu)
    echo "$i,$res,$secs,$(( ${r1:-0} - ${r0:-0} )),$load1,${psi:--}" >> "$CSV"
    case "$res" in
        OK)    ok=$((ok + 1)) ;;
        ABORT) abort=$((abort + 1))
               grep -E "Failed to change state|Aborting" "$log" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-220
               ss -uamn > "$OUT/abort_${i}_ss.txt" 2>&1
               ps -eLo pid,tid,pcpu,comm --sort=-pcpu 2>/dev/null | head -25 > "$OUT/abort_${i}_threads.txt" ;;
        *)     timeout_n=$((timeout_n + 1)) ;;
    esac
    printf '%4d  %-7s %6ss  rcvbuf+%s  load1 %s\n' "$i" "$res" "$secs" "$(( ${r1:-0} - ${r0:-0} ))" "$load1"

    # stop this Nav2 completely before the next one starts
    kill -INT -- "-$NAV" 2>/dev/null
    for _ in $(seq 1 20); do [ -z "$(nav2_procs)" ] && break; sleep 0.5; done
    pkill -9 -f "${IGVC_ROS_PREFIX:-/opt/ros/jazzy}/lib/nav2_" 2>/dev/null
    NAV=""
    sleep 3
done

python3 - "$N" "$ok" "$abort" "$timeout_n" "$LABEL" "$LOAD" "$VARIANT" <<'PY'
import sys
from math import comb
n, ok, ab, to = (int(v) for v in sys.argv[1:5])
label, load, variant = sys.argv[5:8]
def cdf(k, m, p): return sum(comb(m, i) * p**i * (1 - p)**(m - i) for i in range(k + 1))
def bis(f, lo, hi):
    for _ in range(100):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    return (lo + hi) / 2
lo = 0.0 if ab == 0 else bis(lambda p: 0.025 - (1 - cdf(ab - 1, n, p)), 0, 1)
hi = 1.0 if ab == n else bis(lambda p: cdf(ab, n, p) - 0.025, 0, 1)
print("RESULT label=%s load=%s variant=%s n=%d ok=%d aborts=%d timeouts=%d "
      "abort_rate=%.3f 95%%CI=%.3f-%.3f" % (label, load, variant, n, ok, ab, to,
                                            ab / n if n else 0, lo, hi))
PY
echo "CSV: $CSV"
