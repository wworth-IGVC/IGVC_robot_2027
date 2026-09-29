#!/bin/bash
# Is the 2026 competition stack really up? Exit 0 only if every check passes.
#
# docs/COMPETITION_STACK.md, step 8. Runs INSIDE the igvc_comp container:
#
#   docker exec igvc_comp bash /root/comp/check_stack.sh          # check once
#   docker exec igvc_comp bash /root/comp/check_stack.sh --wait   # retry up to 3 min
#
# Compares the live graph with the one measured on 2026-09-24
# (local-notes/ros2_graph_robot_2026-09-24): 32 named nodes plus two tf2
# helper nodes whose names end in random hex, 34 in all. A node missing, a
# node running twice, Nav2 not active or YOLOPv2 not loaded all fail it.
#
# set -u is deliberately absent: it breaks /opt/ros/jazzy/setup.bash.
set -o pipefail

if [ "$1" = "--wait" ]; then
  while [ "$SECONDS" -lt 180 ]; do
    if bash "$0" > /tmp/check_stack_last.txt 2>&1; then
      cat /tmp/check_stack_last.txt
      echo "  (ready after $SECONDS s of waiting)"
      exit 0
    fi
    sleep 5
  done
  cat /tmp/check_stack_last.txt
  echo "  (still not ready after 3 minutes of waiting)"
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source /root/robot_ws/install/setup.bash 2>/dev/null

EXPECTED="
/autonomous_indicator_node
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/collision_monitor
/controller_manager
/controller_server
/diff_drive_controller
/front_zed_camera_x/zed_node
/global_costmap/global_costmap
/igvc_navigator
/igvc_test_bot
/joint_state_broadcaster
/lane_segmentation_node
/left_zed_camera_x/zed_node
/lidar_obstacle_costmap_node
/lifecycle_manager_navigation
/local_costmap/local_costmap
/mission_planner_node
/odom_tf_bridge
/planner_server
/right_zed_camera_x/zed_node
/robot_state_publisher
/route_server
/sllidar_node
/smoother_server
/static_map_to_odom
/twist_stamper
/ublox_gps_node
/velocity_smoother
/waypoint_follower
"

fail=0
pass() { echo "  PASS  $*"; }
bad()  { echo "  FAIL  $*"; fail=1; }

# Capture first, then match: a pipe straight into grep -q can report a false
# negative under pipefail.
timeout 20 ros2 node list > /tmp/check_nodes.txt 2>/tmp/check_nodes.err
nodes=$(grep '^/' /tmp/check_nodes.txt | grep -v '^/_ros2cli' | sort)
total=$(printf '%s\n' "$nodes" | grep -c '^/')

missing=""
for n in $EXPECTED; do
  printf '%s\n' "$nodes" | grep -qx "$n" || missing="$missing $n"
done
if [ -z "$missing" ]; then
  pass "all 32 named nodes of the 2026-09-24 graph are running"
else
  bad "missing nodes:$missing"
fi

dups=$(printf '%s\n' "$nodes" | uniq -d)
if [ -z "$dups" ]; then
  pass "no node is running twice"
else
  bad "running more than once (a second stack, or leftovers): $(echo $dups)"
fi
echo "        nodes in the graph: $total (34 on 2026-09-24)"

if grep -q 'Managed nodes are active' /tmp/robot.log 2>/dev/null; then
  pass "Nav2 lifecycle manager: Managed nodes are active"
else
  bad "Nav2 has not reported 'Managed nodes are active' in /tmp/robot.log yet"
fi

if grep -q 'YOLOPv2 ready' /tmp/robot.log 2>/dev/null; then
  pass "lane_segmentation_node: $(grep -o 'YOLOPv2 ready.*' /tmp/robot.log | tail -n 1 | cut -c1-120)"
else
  bad "lane_segmentation_node has not reported 'YOLOPv2 ready' in /tmp/robot.log yet"
fi

died=$(grep -c 'process has died' /tmp/robot.log 2>/dev/null)
if [ "${died:-0}" = 0 ]; then
  pass "no process has died"
else
  bad "$died process(es) died; see: grep 'process has died' /tmp/robot.log"
fi

echo
if [ "$fail" = 0 ]; then
  echo "  COMPETITION STACK: UP"
else
  echo "  COMPETITION STACK: NOT READY. Straight after launch that is normal:"
  echo "  run this again with --wait. If it still fails, read /tmp/robot.log."
fi
exit "$fail"
