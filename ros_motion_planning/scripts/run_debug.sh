#!/usr/bin/env bash
# run_debug.sh
# 启动所有节点，输出写入 log/ 目录，方便查看和传输

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/log"

source "$WORKSPACE_DIR/devel/setup.bash"
mkdir -p "$LOG_DIR"

# 清理旧日志
rm -f "$LOG_DIR"/*.log

# 记录启动时间
echo "=== 启动时间: $(date) ===" > "$LOG_DIR/run.log"

# ── 1. 生成动态启动文件 ──
echo "[1/4] 生成动态启动文件..."
python "$WORKSPACE_DIR/src/plugins/dynamic_xml_config/main_generate.py" user_config.yaml \
    >> "$LOG_DIR/run.log" 2>&1

# ── 2. 启动 ROS master + Gazebo + RViz（主程序） ──
echo "[2/4] 启动仿真主程序 (Gazebo + RViz)..."
roslaunch sim_env main.launch \
    > "$LOG_DIR/gazebo.log" 2>&1 &
PID_GAZEBO=$!
echo "  PID=$PID_GAZEBO → log/gazebo.log"

# 等待 Gazebo 启动
sleep 8

# ── 3. 启动编码器读取 ──
echo "[3/4] 启动 read_uart.py (编码器读取)..."
rosrun encoder_tools read_uart.py \
    > "$LOG_DIR/read_uart.log" 2>&1 &
PID_READ=$!
echo "  PID=$PID_READ → log/read_uart.log"

sleep 2

# ── 4. 启动里程计 ──
echo "[4/4] 启动 encoder_odom.py (里程计)..."
rosrun encoder_tools encoder_odom.py \
    > "$LOG_DIR/encoder_odom.log" 2>&1 &
PID_ODOM=$!
echo "  PID=$PID_ODOM → log/encoder_odom.log"

# 保存 PID 以便后续 kill
echo "$PID_GAZEBO" > "$LOG_DIR/.pid_gazebo"
echo "$PID_READ" > "$LOG_DIR/.pid_read"
echo "$PID_ODOM" > "$LOG_DIR/.pid_odom"

echo ""
echo "========================================"
echo "  全部已启动，日志文件在 log/ 目录下"
echo "========================================"
echo ""
echo "  查看里程计日志:  tail -f $LOG_DIR/encoder_odom.log"
echo "  查看编码器日志:  tail -f $LOG_DIR/read_uart.log"
echo "  查看所有日志:    tail -f $LOG_DIR/*.log"
echo ""
echo "  发送指令请另开终端运行:"
echo "    cd $WORKSPACE_DIR && source devel/setup.bash"
echo "    rosrun encoder_tools send_uart.py"
echo ""
echo "  停止所有进程:   bash $SCRIPT_DIR/stop_debug.sh"
echo "========================================"
