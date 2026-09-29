#!/bin/bash
# Build the 2026 competition stack inside the igvc_comp container.
#
# docs/COMPETITION_STACK.md, step 6. Runs INSIDE the container, after steps 3
# to 5 have copied in the competition packages (from commit 67a6934),
# zed-description, the YOLOPv2 weights and this folder. Safe to run again.
#
# It changes exactly one thing in the competition code, and only in the
# container's copy: the ODrive CAN hardware plugin becomes ros2_control's
# mock_components/GenericSystem, because there is no CAN bus here. The
# controller nodes and their topics are the same either way. Nothing in the
# repo is touched.
#
# set -u is deliberately absent: it breaks /opt/ros/jazzy/setup.bash.
set -eo pipefail

WS=/root/robot_ws
YOLOPV2_SHA256=f2a8c8374203ae3e67ff9c184e931f763957de92a993b23269e4e721627f1f8c

source /opt/ros/jazzy/setup.bash
cd "$WS"

missing=0
for d in src/igvc_lane_detection src/igvc_test_bringup src/igvc_test_description src/zed-description; do
  if [ ! -f "$d/package.xml" ]; then
    echo "  FAIL  $WS/$d is missing (docs/COMPETITION_STACK.md steps 3 and 4)"
    missing=1
  fi
done
if [ ! -f models/yolopv2.pt ]; then
  echo "  FAIL  $WS/models/yolopv2.pt is missing (docs/COMPETITION_STACK.md step 5)"
  missing=1
fi
[ "$missing" = 0 ] || exit 1
echo "  PASS  competition packages, zed-description and weights are in place"

got=$(sha256sum models/yolopv2.pt | awk '{print $1}')
if [ "$got" != "$YOLOPV2_SHA256" ]; then
  echo "  FAIL  models/yolopv2.pt is not the upstream YOLOPv2 release"
  echo "        expected $YOLOPV2_SHA256"
  echo "        got      $got"
  exit 1
fi
echo "  PASS  YOLOPv2 weights match the recorded SHA256"

X=src/igvc_test_description/urdf/control/ros2_control_info.urdf.xacro
sed -i 's#<plugin>odrive_ros2_control_plugin/ODriveHardwareInterface</plugin>#<plugin>mock_components/GenericSystem</plugin>#' "$X"
if grep -q ODriveHardwareInterface "$X" || ! grep -q 'mock_components/GenericSystem' "$X"; then
  echo "  FAIL  could not swap the ODrive plugin for the mock in $X"
  exit 1
fi
echo "  PASS  ODrive CAN plugin swapped for mock_components/GenericSystem (container copy only)"

echo "        building 4 packages (a few seconds); log in /tmp/comp_build.log"
if ! colcon build --packages-select zed_description igvc_test_description \
      igvc_lane_detection igvc_test_bringup > /tmp/comp_build.log 2>&1; then
  tail -n 30 /tmp/comp_build.log
  echo "  FAIL  colcon build; full log in /tmp/comp_build.log"
  exit 1
fi
summary=$(grep -E '^Summary:' /tmp/comp_build.log || true)
echo "  PASS  colcon build: ${summary:-no summary line}"

# Every `docker exec -it igvc_comp bash` from now on opens a sourced ROS 2
# shell. Ubuntu's .bashrc stops early for non-interactive shells, so these
# lines only affect interactive terminals, which is the point.
if ! grep -q 'IGVC competition stack' /root/.bashrc; then
  cat >> /root/.bashrc <<'EOF'

# IGVC competition stack (added by scripts/competition_stack/setup_in_container.sh)
source /opt/ros/jazzy/setup.bash
source /root/robot_ws/install/setup.bash
export YOLOPV2_WEIGHTS=/root/robot_ws/models/yolopv2.pt
EOF
fi
echo "  PASS  new shells in this container source ROS 2 Jazzy and the workspace"
echo
echo "  Setup complete. Next: docs/COMPETITION_STACK.md step 7."
