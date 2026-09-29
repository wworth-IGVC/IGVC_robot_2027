#!/bin/bash
# Start the 2026 competition stack in the foreground. Ctrl-C stops it.
#
# docs/COMPETITION_STACK.md, step 7. Runs INSIDE the igvc_comp container:
# (setup_in_container.sh is step 6; check_stack.sh is step 8.)
#
#   docker exec -it igvc_comp bash /root/comp/launch_stack.sh
#
# It starts the five hardware-driver stand-ins (driver_standins.py) in the
# background, then the competition's top-level launch exactly as the branch's
# scripts/auton_launch.sh runs it: igvc_fused_drive.launch.py with no
# arguments and YOLOPV2_WEIGHTS set. Output also goes to /tmp/robot.log,
# which check_stack.sh reads.
#
# set -u is deliberately absent: it breaks /opt/ros/jazzy/setup.bash.
set -o pipefail

source /opt/ros/jazzy/setup.bash
source /root/robot_ws/install/setup.bash
export YOLOPV2_WEIGHTS=/root/robot_ws/models/yolopv2.pt

# Two copies of the stack in one container would give every node twice, and
# the graph would look healthy while being meaningless. Nodes started by
# ros2 launch carry --ros-args on their command line; the stand-ins do not,
# so they are looked for by name.
if pgrep -f -- '--ros-args' >/dev/null || pgrep -f driver_standins.py >/dev/null; then
  echo "  The competition stack, or leftovers of an earlier one, is ALREADY"
  echo "  RUNNING in this container. Reset it from PowerShell, then run this again:"
  echo
  echo "    docker restart igvc_comp"
  exit 1
fi

python3 /root/comp/driver_standins.py > /tmp/standins.log 2>&1 &
STANDINS=$!
trap 'kill $STANDINS 2>/dev/null' EXIT

echo "  driver stand-ins started; launching igvc_fused_drive.launch.py"
echo "  Ready when you see BOTH 'Managed nodes are active' (Nav2)"
echo "  and 'YOLOPv2 ready' (lane segmentation), usually within 30 s."
echo "  Check it from another window:"
echo "    docker exec igvc_comp bash /root/comp/check_stack.sh --wait"
echo

# tee ignores SIGINT so that on Ctrl-C it keeps printing ros2 launch's own
# shutdown messages instead of dying first and breaking the pipe.
ros2 launch igvc_test_bringup igvc_fused_drive.launch.py 2>&1 | (trap '' INT; tee /tmp/robot.log)
