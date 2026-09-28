#!/usr/bin/env bash

# Standalone fixed-speed test for the ROS-to-MCU serial path.
# It starts roscore, serial_bridge.py, and encoder_odom.py when necessary.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/log"

DURATION_S="${1:-5}"
LINEAR_MPS="${2:-0.300}"
ANGULAR_RADPS="${3:-0.000}"
RATE_HZ="${4:-20}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyTHS0}"
SERIAL_BAUD="${SERIAL_BAUD:-57600}"

is_number() {
    [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?$ ]]
}

for value in "$DURATION_S" "$LINEAR_MPS" "$ANGULAR_RADPS" "$RATE_HZ"; do
    if ! is_number "$value"; then
        echo "Usage: $0 [duration_s] [linear_mps] [angular_radps] [rate_hz]" >&2
        exit 2
    fi
done

if ! awk -v duration="$DURATION_S" -v linear="$LINEAR_MPS" -v angular="$ANGULAR_RADPS" \
    'BEGIN { exit !(duration > 0 && duration <= 10 && linear >= -0.40 && linear <= 0.40 && angular >= -1.50 && angular <= 1.50) }'; then
    echo "Safety limits: 0 < duration <= 10, |linear| <= 0.40, |angular| <= 1.50" >&2
    exit 2
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "ROS Noetic was not found at /opt/ros/noetic/setup.bash" >&2
    exit 1
fi

if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
    echo "Missing $WORKSPACE_DIR/devel/setup.bash. Build this workspace first." >&2
    exit 1
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
TEST_LOG="$LOG_DIR/serial_speed_test_${STAMP}.log"
ROSCORE_LOG="$LOG_DIR/serial_speed_test_${STAMP}_roscore.log"
BRIDGE_LOG="$LOG_DIR/serial_speed_test_${STAMP}_bridge.log"
ODOM_NODE_LOG="$LOG_DIR/serial_speed_test_${STAMP}_encoder_odom.log"
ODOM_LOG="$LOG_DIR/serial_speed_test_${STAMP}_odom.csv"
WHEEL_LOG="$LOG_DIR/serial_speed_test_${STAMP}_wheel.csv"

LEFT_WHEEL=$(awk -v v="$LINEAR_MPS" -v w="$ANGULAR_RADPS" 'BEGIN { printf "%.3f", v - w * 0.45 / 2.0 }')
RIGHT_WHEEL=$(awk -v v="$LINEAR_MPS" -v w="$ANGULAR_RADPS" 'BEGIN { printf "%.3f", v + w * 0.45 / 2.0 }')
ZERO_TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
TEST_TWIST="{linear: {x: $LINEAR_MPS, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_RADPS}}"

ROSCORE_PID=""
BRIDGE_PID=""
ENCODER_PID=""
PUBLISHER_PID=""
ODOM_PID=""
WHEEL_PID=""

stop_pid() {
    local pid="$1"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    local exit_code=$?
    rostopic pub -1 /cmd_vel geometry_msgs/Twist "$ZERO_TWIST" >/dev/null 2>&1 || true
    stop_pid "$PUBLISHER_PID"
    stop_pid "$ODOM_PID"
    stop_pid "$WHEEL_PID"
    stop_pid "$ENCODER_PID"
    stop_pid "$BRIDGE_PID"
    stop_pid "$ROSCORE_PID"
    exit "$exit_code"
}
trap cleanup EXIT INT TERM

wait_for_master() {
    local attempt
    for attempt in $(seq 1 50); do
        if rostopic list >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

exec > >(tee -a "$TEST_LOG") 2>&1

echo "serial speed test started: $(date -Is)"
echo "command: v=$LINEAR_MPS m/s, w=$ANGULAR_RADPS rad/s, rate=$RATE_HZ Hz, duration=$DURATION_S s"
echo "expected serial payload: l:$LEFT_WHEEL,r:$RIGHT_WHEEL"
echo "serial port: $SERIAL_PORT @ $SERIAL_BAUD"
echo "test log: $TEST_LOG"

if ! rostopic list >/dev/null 2>&1; then
    echo "starting private roscore"
    roscore > "$ROSCORE_LOG" 2>&1 &
    ROSCORE_PID=$!
    if ! wait_for_master; then
        echo "roscore did not become ready; see $ROSCORE_LOG" >&2
        exit 1
    fi
else
    echo "using existing ROS master"
fi

if rosnode list 2>/dev/null | grep -qE '^/serial_bridge$'; then
    echo "A serial_bridge node is already running. Stop the debug stack before this standalone test." >&2
    exit 1
fi

if [[ ! -e "$SERIAL_PORT" ]]; then
    echo "Serial port $SERIAL_PORT does not exist." >&2
    exit 1
fi

echo "starting serial bridge"
rosrun encoder_tools serial_bridge.py _port:="$SERIAL_PORT" _baud:="$SERIAL_BAUD" \
    > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

echo "starting encoder odometry"
rosrun encoder_tools encoder_odom.py > "$ODOM_NODE_LOG" 2>&1 &
ENCODER_PID=$!

sleep 1
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "serial_bridge.py exited; see $BRIDGE_LOG" >&2
    exit 1
fi
if ! kill -0 "$ENCODER_PID" 2>/dev/null; then
    echo "encoder_odom.py exited; see $ODOM_NODE_LOG" >&2
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

stop_pid "$PUBLISHER_PID"
PUBLISHER_PID=""
rostopic pub -1 /cmd_vel geometry_msgs/Twist "$ZERO_TWIST" >/dev/null 2>&1 || true
sleep 0.5

echo "fixed command complete: $(date -Is)"
echo "bridge log: $BRIDGE_LOG"
echo "odom node log: $ODOM_NODE_LOG"
echo "odom samples: $ODOM_LOG"
echo "wheel samples: $WHEEL_LOG"
echo "the robot has been commanded to stop"
