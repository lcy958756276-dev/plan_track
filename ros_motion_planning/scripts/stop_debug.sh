#!/usr/bin/env bash
# stop_debug.sh
# 停止 run_debug.sh 启动的所有进程

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/log"

echo "正在停止所有进程..."

for pid_file in "$LOG_DIR"/.pid_*; do
    if [ -f "$pid_file" ]; then
        pid=$(cat "$pid_file")
        name=$(basename "$pid_file" | sed 's/.pid_//')
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            echo "  已停止 $name (PID=$pid)"
        else
            echo "  $name (PID=$pid) 已结束"
        fi
        rm -f "$pid_file"
    fi
done

# 额外清理可能残留的 ROS 节点和参数
killall -q roslaunch gzserver gzclient 2>/dev/null

# 清理物理车模式标志位（run_debug.sh 设置的）
rm -f /tmp/.use_encoder_odom
echo "  已清理 /tmp/.use_encoder_odom 标志位"

echo "清理完成"
