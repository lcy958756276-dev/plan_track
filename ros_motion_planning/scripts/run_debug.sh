#!/usr/bin/env bash
# run_debug.sh
# 启动所有节点（含 Gazebo，但 Gazebo 不发布 odom TF，由 encoder_odom.py 接管）
# 输出写入 log/ 目录
#
# 与 main.sh 的关系:
#   - main.sh: 原版 roslaunch（Gazebo 正常发布 odom TF，用于纯仿真）
#   - run_debug.sh: 设置 /tmp/.use_encoder_odom 标志位，Gazebo 跳过 odom TF
#   - stop_debug.sh: 清理此标志位

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/log"

source "$WORKSPACE_DIR/devel/setup.bash"
mkdir -p "$LOG_DIR"

# 强制清理残留的 Gazebo 进程 + ROS master
# 避免新旧 gzserver 节点名冲突导致 ROS 服务无法加载
killall -9 gzserver gzclient rosmaster 2>/dev/null
sleep 2

# 清理旧日志
rm -f "$LOG_DIR"/*.log

# 记录启动时间
echo "=== 启动时间: $(date) ===" > "$LOG_DIR/run.log"

# ── 1. 生成动态启动文件 ──
echo "[1/6] 生成动态启动文件..."
python "$WORKSPACE_DIR/src/plugins/dynamic_xml_config/main_generate.py" user_config.yaml \
    >> "$LOG_DIR/run.log" 2>&1

# ── 标志位：告诉 Gazebo 不发布 odom TF（物理车模式）──
# main.sh 没有此文件 → Gazebo 正常行为
echo "[2/6] 设置 use_encoder_odom 标志位..."
touch /tmp/.use_encoder_odom
echo "  /tmp/.use_encoder_odom → Gazebo diff_drive 将跳过 odom TF"

# ── 3. 加载机器人模型 ──
echo "[3/6] 加载 robot_description + 启动核心节点 ..."

# 从 xacro 加载 robot description（传 use_encoder_odom:=true）
ROBOT_XACRO="$WORKSPACE_DIR/src/sim_env/urdf/turtlebot3_waffle/turtlebot3_waffle.xacro"
rosparam set robot_description "$(xacro --inorder "$ROBOT_XACRO" use_encoder_odom:=true)"
echo "  robot_description 已加载（物理车模式 → publishOdomTF=false）"

# robot_state_publisher（发布 URDF 中固定关节的 TF：base_footprint → base_link → ...）
rosrun robot_state_publisher robot_state_publisher \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_RSP=$!
echo "  robot_state_publisher PID=$PID_RSP"

sleep 1

# map_server（提供背景地图）
MAP_FILE="$WORKSPACE_DIR/src/sim_env/maps/warehouse/warehouse.yaml"
rosrun map_server map_server "$MAP_FILE" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_MAP=$!
echo "  map_server PID=$PID_MAP"

sleep 1

# 静态 map → odom TF（让 RViz 在 map 固定帧下能看到小车移动）
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_TF=$!
echo "  static_transform_publisher (map→odom) PID=$PID_TF"

sleep 1

# ── 4. 启动 Gazebo（仓库环境）──
echo "[4/6] 启动 Gazebo ..."
WORLD_FILE="$WORKSPACE_DIR/src/sim_env/worlds/warehouse.world"

# 先启动 Gazebo 服务器
rosrun gazebo_ros gzserver "$WORLD_FILE" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_GZSERVER=$!
echo "  gazebo server PID=$PID_GZSERVER"

sleep 3

# Gazebo GUI
rosrun gazebo_ros gzclient \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_GZCLIENT=$!
echo "  gazebo client PID=$PID_GZCLIENT"

sleep 2

# 在 Gazebo 中生成机器人模型（使用已加载的 robot_description）
# 注意：此时 diff_drive 插件 publishOdomTF=false，不会与 encoder_odom 冲突
rosrun gazebo_ros spawn_model -urdf \
    -param robot_description \
    -model turtlebot3_waffle \
    -x 0.0 -y 0.0 -z 0.0 \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_SPAWN=$!
echo "  机器人已在 Gazebo 中生成 (use_encoder_odom=true → 不发布 odom TF)"

sleep 2

# ├── RViz ──
RVIZ_FILE="$WORKSPACE_DIR/src/sim_env/rviz/sim_env.rviz"
rosrun rviz rviz -d "$RVIZ_FILE" \
    >> "$LOG_DIR/run.log" 2>&1 &
PID_RVIZ=$!
echo "  rviz PID=$PID_RVIZ"

sleep 2

# ── 恢复 ROS 时间为真实时间 ──
# Gazebo 的 gzserver 内部会设 /use_sim_time=true，导致
# rospy.Time.now() 返回仿真时间（卡在 0），encoder_odom 无法计算 dt。
# 这里显式改回 false，让 read_uart / encoder_odom 用真实时钟。
rosparam set /use_sim_time false
echo "  /use_sim_time → false（编码器/里程计使用真实时钟）"

# ── 5. 启动编码器读取 ──
echo "[5/6] 启动 read_uart.py (编码器读取)..."
rosrun encoder_tools read_uart.py \
    > "$LOG_DIR/read_uart.log" 2>&1 &
PID_READ=$!
echo "  PID=$PID_READ → log/read_uart.log"

sleep 2

# ── 6. 启动里程计 ──
echo "[6/6] 启动 encoder_odom.py (里程计)..."
rosrun encoder_tools encoder_odom.py \
    > "$LOG_DIR/encoder_odom.log" 2>&1 &
PID_ODOM=$!
echo "  PID=$PID_ODOM → log/encoder_odom.log"

sleep 1

# ── 7. 启动 Gazebo 同步桥接 ──
echo "[7/7] 启动 gazebo_sync.py (里程计→Gazebo 同步 + LaserScan 时间戳修复)..."
rosrun encoder_tools gazebo_sync.py \
    > "$LOG_DIR/gazebo_sync.log" 2>&1 &
PID_SYNC=$!
echo "  PID=$PID_SYNC → log/gazebo_sync.log"

# 保存 PID
echo "$PID_MAP"      > "$LOG_DIR/.pid_map"
echo "$PID_RSP"      > "$LOG_DIR/.pid_rsp"
echo "$PID_TF"       > "$LOG_DIR/.pid_tf"
echo "$PID_GZSERVER" > "$LOG_DIR/.pid_gzserver"
echo "$PID_GZCLIENT" > "$LOG_DIR/.pid_gzclient"
echo "$PID_RVIZ"     > "$LOG_DIR/.pid_rviz"
echo "$PID_READ"     > "$LOG_DIR/.pid_read"
echo "$PID_ODOM"     > "$LOG_DIR/.pid_odom"
echo "$PID_SYNC"     > "$LOG_DIR/.pid_sync"

echo ""
echo "========================================"
echo "  全部已启动，日志文件在 log/ 目录下"
echo ""
echo "  Gazebo 已启动（不干扰 encoder_odom 的 TF）"
echo "  小车定位由物理编码器驱动"
echo "========================================"
echo ""
echo "  查看里程计日志:  tail -f $LOG_DIR/encoder_odom.log"
echo "  查看编码器日志:  tail -f $LOG_DIR/read_uart.log"
echo ""
echo "  发送指令请另开终端运行:"
echo "    cd $WORKSPACE_DIR && source devel/setup.bash"
echo "    rosrun encoder_tools send_uart.py"
echo ""
echo "  停止所有进程:   bash $SCRIPT_DIR/stop_debug.sh"
echo "========================================"
