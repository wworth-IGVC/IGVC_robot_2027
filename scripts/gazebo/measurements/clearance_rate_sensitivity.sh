#!/usr/bin/env bash
# Re-score one autonomy pose log at decreasing sample rates, to show how much
# of the lane-clearance margin is an artifact of sampling too slowly. Produced
# the 2.4x figure: +0.093 m at 15.8 Hz against +0.227 m at 2 Hz.
# Run inside the Gazebo container AFTER an autonomy_check.sh run, or give it a
# saved log:   bash clearance_rate_sensitivity.sh [track_log]
#
# Scores through autonomy_check.sh's ANALYSE_ONLY mode, so it is the same code
# that grades a live run. (Until 2026-10-07 this cut the Python block out of
# autonomy_check.sh with awk and called it by hand, which broke as soon as the
# block gained arguments.)
set -o pipefail
REPO=/root/ros2_ws/src/IGVC_robot_2026
LOG="${1:-/tmp/track_log.txt}"
N=$(wc -l < "$LOG")
echo "full log: $N samples"
for STRIDE in 1 2 4 8; do
  awk -v s=$STRIDE 'NR % s == 1 % s' "$LOG" > /tmp/ds.txt
  M=$(wc -l < /tmp/ds.txt)
  printf "\n--- every %dth sample (%d rows) ---\n" "$STRIDE" "$M"
  ANALYSE_ONLY=/tmp/ds.txt bash "$REPO/scripts/gazebo/autonomy_check.sh" 2>&1 \
    | grep -E "sample rate|centreline deviation|lane clearance|yaw to lane|LANE  "
done
