#!/usr/bin/env bash

# Fixed-command test for the ROS-to-MCU serial path. Run this only with the
# debug stack already running and no active navigation goal.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/log"

DURATION_S="${1:-5}"
LINEAR_MPS="${2:-0.300}"
ANGULAR_RADPS="${3:-0.000}"
RATE_HZ="${4:-20}"

is_number() {
    [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?$ ]]
}

for value in "$DURATION_S" "$LINEAR_MPS" "$ANGULAR_RADPS" "$RATE_HZ"; do
    if ! is_number "$value"; then
        echo "All arguments must be numeric: duration_s linear_mps angular_radps rate_hz" >&2
        exit 2
    fi
done

if ! awk -v duration="$DURATION_S" -v linear="$LINEAR_MPS" -v angular="$ANGULAR_RADPS" \
    'BEGIN { exit !(duration > 0 && duration <= 10 && linear >= -0.40 && linear <= 0.40 && angular >= -1.50 && angular <= 1.50) }'; then
    echo "Safety limits: 0 < duration <= 10, |linear| <= 0.40, |angular| <= 1.50" >&2
    exit 2
fi

mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
TEST_LOG="$LOG_DIR/serial_speed_test_${STAMP}.log"
ODOM_LOG="$LOG_DIR/serial_speed_test_${STAMP}_odom.csv"
WHEEL_LOG="$LOG_DIR/serial_speed_test_${STAMP}_wheel.csv"

LEFT_WHEEL=$(awk -v v="$LINEAR_MPS" -v w="$ANGULAR_RADPS" 'BEGIN { printf "%.3f", v - w * 0.45 / 2.0 }')
RIGHT_WHEEL=$(awk -v v="$LINEAR_MPS" -v w="$ANGULAR_RADPS" 'BEGIN { printf "%.3f", v + w * 0.45 / 2.0 }')
ZERO_TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
TEST_TWIST="{linear: {x: $LINEAR_MPS, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_RADPS}}"

PUBLISHER_PID=""
ODOM_PID=""
WHEEL_PID=""

cleanup() {
    local exit_code=$?
    for pid in "$PUBLISHER_PID" "$ODOM_PID" "$WHEEL_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "$ZERO_TWIST" >/dev/null 2>&1 || true
    exit "$exit_code"
}
trap cleanup EXIT INT TERM

exec > >(tee -a "$TEST_LOG") 2>&1

echo "serial speed test started: $(date -Is)"
echo "command: v=$LINEAR_MPS m/s, w=$ANGULAR_RADPS rad/s, rate=$RATE_HZ Hz, duration=$DURATION_S s"
echo "expected serial payload: l:$LEFT_WHEEL,r:$RIGHT_WHEEL"
echo "test log: $TEST_LOG"

if ! rostopic list >/dev/null 2>&1; then
    echo "ROS master is unavailable. Start run_debug.sh before this test." >&2
    exit 1
fi

echo "cmd_vel publishers before test:"
rostopic info /cmd_vel || true

if ! rostopic list | grep -qx '/wheel_ticks'; then
    echo "/wheel_ticks is unavailable. serial_bridge.py must be running." >&2
    exit 1
fi

rostopic echo -p /odom > "$ODOM_LOG" 2>&1 &
ODOM_PID=$!
rostopic echo -p /wheel_velocities > "$WHEEL_LOG" 2>&1 &
WHEEL_PID=$!

sleep 0.5
echo "publishing fixed command now"
rostopic pub -r "$RATE_HZ" /cmd_vel geometry_msgs/Twist "$TEST_TWIST" >/dev/null 2>&1 &
PUBLISHER_PID=$!
sleep "$DURATION_S"

kill "$PUBLISHER_PID" 2>/dev/null || true
wait "$PUBLISHER_PID" 2>/dev/null || true
PUBLISHER_PID=""
rostopic pub -1 /cmd_vel geometry_msgs/Twist "$ZERO_TWIST" >/dev/null 2>&1 || true
sleep 0.5

echo "fixed command complete: $(date -Is)"
echo "odom samples: $ODOM_LOG"
echo "wheel samples: $WHEEL_LOG"
echo "check the matching interval in log/serial_bridge.log for the exact bytes written to MCU"
