#!/usr/bin/env bash
# Re-score one autonomy pose log at decreasing sample rates, to show how much
# of the lane-clearance margin is an artifact of sampling too slowly. Produced
# the 2.4x figure: +0.093 m at 15.8 Hz against +0.227 m at 2 Hz.
# Run inside the Gazebo container AFTER an autonomy_check.sh run.
set -o pipefail
REPO=/root/ros2_ws/src/IGVC_robot_2026
awk '/^python3 - /{f=1;next} /^PY$/{f=0} f' "$REPO/scripts/gazebo/autonomy_check.sh" > /tmp/score.py
N=$(wc -l < /tmp/track_log.txt)
echo "full log: $N samples"
for STRIDE in 1 2 4 8; do
  awk -v s=$STRIDE 'NR % s == 1' /tmp/track_log.txt > /tmp/ds.txt
  M=$(wc -l < /tmp/ds.txt)
  printf "\n--- every %dth sample (%d rows, ~%.1f Hz) ---\n" "$STRIDE" "$M" "$(python3 -c "print($M/52.5)")"
  python3 /tmp/score.py "$REPO/IGVC_track_generator/track_points.json" /tmp/ds.txt 2.0 0.2311 "$REPO/scripts/gazebo" 2>&1 \
    | grep -E "centreline deviation|lane clearance|yaw to lane|IN LANE"
done
