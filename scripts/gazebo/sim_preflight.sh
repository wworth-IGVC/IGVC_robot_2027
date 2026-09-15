#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# sim_preflight.sh - refuse to start a second simulator on top of a first.
#
# Source this, then call sim_preflight.
#
# WHY THIS EXISTS
#
# On 2026-09-15 a test run produced a robot that drove nicely and then began
# turning in circles, and a Nav2 lifecycle bringup that aborted with
# "Failed to change state for node: smoother_server". Both were investigated
# as navigation problems. Neither was. `ps` showed SIX gz sim processes, TWO
# rviz2 and THREE parameter_bridge instances: leftovers from earlier runs whose
# cleanup had not actually worked.
#
# Two simulators on one ROS domain is not a degraded version of one. Both
# publish /clock, so time jumps backwards and forwards between them. Both
# publish /odom and /tf for a robot named igvc_robot, so the TF tree has two
# disagreeing answers for where the robot is, and consumers get whichever
# arrived last. Nav2 then plans against a pose that teleports, which looks
# exactly like a controller tuning problem and is not one. Meanwhile the second
# Gazebo is rendering the same course on the same GPU, so everything is
# starved and lifecycle transitions time out.
#
# This is the same family of mistake as `docker compose run` starting a new
# container each time, recorded in docs/GAZEBO_SETUP.md 8.1. The lesson is
# the same: check what is already running before adding to it.
#
# Usage:
#     source "$(dirname "$0")/sim_preflight.sh"
#     sim_preflight            # refuse if anything is running
#     FORCE=1 sim_preflight    # kill it and carry on (what the tests do)
# ---------------------------------------------------------------------------

# Match on where the executable lives, not on a list of node names.
#
# The name-list version of this function missed five nodes and let a second
# gazebo_odom_shim, robot_state_publisher, track_ground_truth_node,
# gt_nav_bridge_node and localization_node survive a "successful" cleanup,
# which put two publishers back on /odom - the exact failure the whole file
# exists to prevent. Killing `ros2 launch` does NOT reap its children: they are
# reparented to init and carry on happily.
#
# Every node in this stack is executed from /opt/ros/jazzy/lib or from
# /root/ros2_ws/install, and nothing else in this container runs from either,
# so matching the path catches all of them and needs no maintenance when a
# node is added.
_sim_procs () {
    # -f matches the full command line. Safe from self-matching here because a
    # script's own command line is "bash /path/to/script.sh", which contains
    # none of these patterns. It is NOT safe from an inline `bash -c` whose
    # command string happens to contain them.
    pgrep -f '/opt/ros/jazzy/lib/'     2>/dev/null
    pgrep -f '/root/ros2_ws/install/'  2>/dev/null
    pgrep -f 'gz sim'                  2>/dev/null
    pgrep -f 'ros2 launch'             2>/dev/null
}

sim_preflight () {
    local found
    found=$(_sim_procs | sort -u | wc -l)
    if [ "$found" -eq 0 ]; then
        return 0
    fi

    echo "=============================================================="
    echo " A simulator is ALREADY RUNNING in this container: $found processes"
    echo "=============================================================="
    ps -eo pid,etime,cmd 2>/dev/null \
        | grep -E 'gz sim|rviz2|parameter_bridge|ros2 launch' \
        | grep -v grep | head -12 | sed 's/^/  /'
    echo
    echo " Two simulators on one ROS domain both publish /clock, /odom and"
    echo " /tf for the same robot. Time and pose then jump between them and"
    echo " the navigation stack plans against a pose that teleports. Results"
    echo " from such a run mean nothing, however plausible they look."

    if [ "${FORCE:-0}" = "1" ]; then
        echo
        echo " FORCE=1, so cleaning up and continuing."
        # Kill the launcher first so it stops respawning, then everything it
        # left behind. The second pass is SIGKILL because gz sim and rviz2
        # both ignore SIGTERM often enough to matter.
        pkill -f 'ros2 launch' 2>/dev/null
        sleep 1
        pkill -f '/opt/ros/jazzy/lib/'    2>/dev/null
        pkill -f '/root/ros2_ws/install/' 2>/dev/null
        pkill -f 'gz sim'                 2>/dev/null
        sleep 4
        pkill -9 -f '/opt/ros/jazzy/lib/'    2>/dev/null
        pkill -9 -f '/root/ros2_ws/install/' 2>/dev/null
        pkill -9 -f 'gz sim'                 2>/dev/null
        sleep 2
        local left
        left=$(_sim_procs | sort -u | wc -l)
        if [ "$left" -ne 0 ]; then
            echo " STILL $left processes after cleanup:"
            ps -eo pid,cmd 2>/dev/null | grep -E '/opt/ros/jazzy/lib/|/root/ros2_ws/install/|gz sim' \
                | grep -v grep | head -8 | sed 's/^/   /'
            echo " Restart the container:"
            echo "   docker compose -f docker-compose.windows.yml restart igvc_gazebo"
            return 1
        fi
        echo " Clean."
        return 0
    fi

    echo
    echo " Re-run with FORCE=1 to kill it, or restart the container:"
    echo "   docker compose -f docker-compose.windows.yml restart igvc_gazebo"
    return 1
}
