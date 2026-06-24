#!/usr/bin/env bash
# build_map.sh
# 【在 Jetson 上运行】启动 Gazebo + 小车 + gmapping 建图
# 所有输出写入 log/ 目录

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GAZEBO_DIR="$SCRIPT_DIR"
LOG_DIR="$GAZEBO_DIR/log"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../ros_motion_planning" && pwd 2>/dev/null)"

# 自动探测 workspace
if [ -z "$WORKSPACE_DIR" ] || [ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
    for try_dir in \
        "$SCRIPT_DIR/../ros_motion_planning" \
        "$HOME/plan_track/ros_motion_planning" \
        "$HOME/robot_graduation/ros_motion_planning" \
        "$HOME/ros_motion_planning"; do
        if [ -f "$try_dir/devel/setup.bash" ]; then
            WORKSPACE_DIR="$try_dir"
            break
        fi
    done
fi

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/build.log"
exec > "$LOG_FILE" 2>&1

source "$WORKSPACE_DIR/devel/setup.bash"

# 使用仿真时间（让 mapper 的 TF 和 Gazebo 激光时间戳对齐）
rosparam set /use_sim_time true

echo "=== build_map.sh 启动 ==="
echo "日志目录: $LOG_DIR"
date

# ── 清理 ──
echo "[清理] 杀掉残留进程..."
killall -9 gzserver gzclient 2>/dev/null
pkill -f "gazebo_mapper" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "joint_state_publisher" 2>/dev/null
sleep 2
if pgrep -x gzserver > /dev/null 2>&1; then
    killall -9 gzserver 2>/dev/null
    sleep 1
fi
echo "清理完成"

# ── 1. 加载车模型 ──
echo "[1] 加载 robot_description..."

SIM_ENV_PATH=$(rospack find sim_env 2>/dev/null)
echo "sim_env 路径: $SIM_ENV_PATH"

if [ -z "$SIM_ENV_PATH" ]; then
    echo "错误: 找不到 sim_env 包"
    exit 1
fi

MESH_FILE="$SIM_ENV_PATH/urdf/my_car/meshes/base_link.STL"
if [ ! -f "$MESH_FILE" ]; then
    echo "mesh 不在 sim_env 内，检查本地..."
    MESH_DIR="$GAZEBO_DIR/urdf/my_car/meshes"
    if [ -f "$MESH_DIR/base_link.STL" ]; then
        echo "使用本地 mesh: $MESH_DIR"
        SIM_ENV_PATH="$GAZEBO_DIR/urdf/my_car"
        sed "s|package://sim_env/urdf/my_car|$SIM_ENV_PATH|g" \
            "$GAZEBO_DIR/urdf/my_car/my_car.urdf" | rosparam set robot_description -
    else
        echo "错误: 本地也无 mesh 文件"
        exit 1
    fi
else
    echo "mesh 正常: $MESH_FILE"
    rosparam set robot_description "$(cat "$GAZEBO_DIR/urdf/my_car/my_car.urdf")"
fi

echo "robot_description 已加载"

# ── 2. 启动 Gazebo（使用 gz_debug 命名空间避免冲突）──
echo "[2] 启动 Gazebo..."
rosrun gazebo_ros gzserver "$GAZEBO_DIR/worlds/final.world" __name:=gz_debug \
    > "$LOG_DIR/gzserver.log" 2>&1 &
PID_GZ=$!
echo "gzserver PID=$PID_GZ"

for i in $(seq 1 30); do
    if rosservice list 2>/dev/null | grep -q "/gz_debug/set_model_state"; then
        echo "Gazebo 就绪（第 ${i} 秒）"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "超时: Gazebo 未就绪"
    fi
    sleep 1
done

# 等待仿真时钟开始运行
echo "  等待仿真时钟..."
for i in $(seq 1 15); do
    CLOCK_SEC=$(rostopic echo /clock -n1 2>/dev/null | grep "secs" | head -1 | awk '{print $2}')
    if [ -n "$CLOCK_SEC" ] && [ "$CLOCK_SEC" != "0" ]; then
        echo "  ✅ 仿真时钟运行中 (t=${CLOCK_SEC}s)"
        break
    fi
    sleep 1
done

# 时钟就绪后才启动 RSP（避免时间戳为 0 的 TF 导致 RViz 错乱）
rosrun robot_state_publisher robot_state_publisher \
    > "$LOG_DIR/rsp.log" 2>&1 &
PID_RSP=$!
echo "robot_state_publisher PID=$PID_RSP"

rosrun joint_state_publisher joint_state_publisher \
    > "$LOG_DIR/jsp.log" 2>&1 &
PID_JSP=$!
echo "joint_state_publisher PID=$PID_JSP"
sleep 1

rosrun gazebo_ros gzclient \
    > "$LOG_DIR/gzclient.log" 2>&1 &
PID_GUI=$!
echo "gzclient PID=$PID_GUI"
sleep 3

# ── 3. 生成小车 ──
echo "[3] spawn_model..."
rosrun gazebo_ros spawn_model -urdf \
    -param robot_description \
    -model my_car \
    -gazebo_namespace /gz_debug \
    -x 0.0 -y 0.0 -z 0.0 \
    -Y 3.14159 \
    > "$LOG_DIR/spawn.log" 2>&1
SPAWN_EXIT=$?
echo "spawn_model exit=$SPAWN_EXIT (朝向 180°)"

sleep 3

# 验证模型是否存在
echo "=== 验证模型 ==="
rosservice call /gz_debug/get_model_state '{model_name: "my_car"}' 2>/dev/null | head -5 || echo "get_model_state 失败（模型可能不存在）"
rosservice call /gz_debug/get_world_properties 2>/dev/null | grep "model_names" -A 10 || echo "get_world_properties 失败"

# ── 4. 启动 mapper ──
echo "[4] 启动 gazebo_mapper.py..."
sleep 1  # 等模型就绪后立即启动 mapper
PYTHONUNBUFFERED=1 python3 "$GAZEBO_DIR/scripts/gazebo_mapper.py" \
    --gazebo-namespace /gz_debug \
    --init-yaw-offset 0 \
    --gazebo-yaw-offset 3.14159 \
    > "$LOG_DIR/mapper.log" 2>&1 &
PID_MAPPER=$!
echo "mapper PID=$PID_MAPPER"
sleep 2

# ── 5. gmapping ──
echo "[5] 启动 gmapping..."
rosrun gmapping slam_gmapping scan:=/scan _odom_frame:=odom _map_update_interval:=1.0 \
    > "$LOG_DIR/gmapping.log" 2>&1 &
PID_GMAPPING=$!
echo "gmapping PID=$PID_GMAPPING"
sleep 2

# ── 6. RViz ──
echo "[6] 启动 RViz..."
rosrun rviz rviz -d "$GAZEBO_DIR/rviz/sim_env.rviz" \
    > "$LOG_DIR/rviz.log" 2>&1 &
PID_RVIZ=$!
echo "rviz PID=$PID_RVIZ"

echo ""
echo "=== 全部启动完毕 ==="
echo "查看日志: cat $LOG_FILE"
echo "查看 spawn 日志: cat $LOG_DIR/spawn.log"
echo "查看 gzserver 日志: cat $LOG_DIR/gzserver.log"
echo "查看 mapper 日志: cat $LOG_DIR/mapper.log"
