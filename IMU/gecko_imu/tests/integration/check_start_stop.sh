#!/usr/bin/env bash
# Start, restart and stop the whole system without touching the Wi-Fi.
# It deliberately keeps the caller's environment, including an active Conda.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export GECKO_SKIP_HOTSPOT=1 GECKO_HEADLESS=1 ROS_DOMAIN_ID=77 IGN_PARTITION=gecko_test

PATTERNS=(
    '[g]ecko_twin_node.py'
    '[w]all_climb_manager.py'
    '[u]dp_imu_receiver.py'
    '[i]mu_displacement_estimator.py'
    'models/gecko/[r]obot_state_publisher\.yaml'
)

die() { echo "FAIL: $*" >&2; "$ROOT/start_gecko_pc.sh" stop >/dev/null 2>&1; exit 1; }

check_single_instances() {
    local pattern count
    for pattern in "${PATTERNS[@]}"; do
        count="$(pgrep -fc "$pattern")"
        [[ "$count" == 1 ]] || die "$pattern has $count processes"
    done
    count="$(pgrep -xc rviz2)"
    [[ "$count" == 1 ]] || die "rviz2 has $count processes"
    pgrep -af '[g]ecko_twin_node.py' | grep -q '/usr/bin/python3 ' ||
        die 'twin node is not using the system Python'
}

wait_for_panel() {
    local _ info
    for _ in {1..60}; do
        info="$("$ROOT/tests/run_ros.sh" ros2 topic info /gecko/cmd/drive 2>/dev/null || true)"
        if grep -q 'Publisher count: 1' <<<"$info"; then
            return 0
        fi
        sleep 1
    done
    die 'the RViz panel never published /gecko/cmd/drive'
}

XVFB_BEFORE="$(pgrep -xc Xvfb)"

# A robot_state_publisher from another project must survive start and stop.
/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher \
    --ros-args -r __node:=other_project_rsp -p robot_description:='<robot name="other"><link name="a"/></robot>' \
    >/dev/null 2>&1 &
OTHER_RSP=$!
trap 'kill "$OTHER_RSP" 2>/dev/null' EXIT
sleep 2

"$ROOT/start_gecko_pc.sh" start >/dev/null || die 'first start failed'
wait_for_panel
check_single_instances

# Reopening RViz must not restart the twin.
TWIN_PID="$(pgrep -f '[g]ecko_twin_node.py')"
pkill -x rviz2
sleep 3
"$ROOT/start_gecko_pc.sh" rviz >/dev/null || die 'rviz subcommand failed'
wait_for_panel
check_single_instances
[[ "$(pgrep -f '[g]ecko_twin_node.py')" == "$TWIN_PID" ]] || die 'reopening RViz restarted the twin'

"$ROOT/start_gecko_pc.sh" start >/dev/null || die 'second start failed'
wait_for_panel
check_single_instances

"$ROOT/start_gecko_pc.sh" stop >/dev/null || die 'first stop failed'
"$ROOT/start_gecko_pc.sh" stop >/dev/null || die 'second stop failed'
sleep 2

for pattern in "${PATTERNS[@]}" '[i]gn gazebo' '[p]arameter_bridge'; do
    pgrep -f "$pattern" >/dev/null && die "$pattern is still running after stop"
done
pgrep -x rviz2 >/dev/null && die 'rviz2 is still running after stop'
[[ "$(pgrep -xc Xvfb)" == "$XVFB_BEFORE" ]] || die 'stop left an Xvfb server running'
kill -0 "$OTHER_RSP" 2>/dev/null || die 'stop killed a robot_state_publisher from another project'

echo start_stop_ok
